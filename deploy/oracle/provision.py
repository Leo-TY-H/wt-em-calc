"""Create the EM site's Oracle Always Free Arm VM and minimal public network.

Run using the dedicated OCI Python environment after ~/.oci/config is set up:
    python deploy/oracle/provision.py --plan
    python deploy/oracle/provision.py --apply
Only the account's home region, A1 shape, 2 OCPUs, 12 GB RAM and 50 GB boot disk
are used. Existing resources with this tool's names are reused on a retry.
"""

import argparse
import ipaddress
from pathlib import Path
import sys
import time
from urllib.request import urlopen

import oci


NAME = "wt-em-free"
SHAPE = "VM.Standard.A1.Flex"
OCPUS = 2
MEMORY_GB = 12
BOOT_GB = 50


def all_pages(call, *args, **kwargs):
    return oci.pagination.list_call_get_all_results(call, *args, **kwargs).data


def wait_for(call, wanted, timeout=600):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        item = call().data
        if item.lifecycle_state == wanted:
            return item
        if item.lifecycle_state in ("FAILED", "TERMINATED"):
            raise RuntimeError(f"Oracle resource entered {item.lifecycle_state}")
        time.sleep(5)
    raise TimeoutError(f"Oracle resource did not reach {wanted}")


def named(items, name):
    found = [x for x in items if x.display_name == name and x.lifecycle_state not in ("TERMINATED", "DELETED")]
    if len(found) > 1:
        raise RuntimeError(f"Multiple resources named {name}; refusing to guess")
    return found[0] if found else None


def check_allowance(identity, compute, block, tenancy, domains):
    compartments = [tenancy] + [x.id for x in all_pages(
        identity.list_compartments, tenancy, compartment_id_in_subtree=True,
        access_level="ANY") if x.lifecycle_state == "ACTIVE"]
    instances, storage_gb = [], 0
    for compartment in compartments:
        instances += [x for x in all_pages(compute.list_instances, compartment)
                      if x.lifecycle_state != "TERMINATED"]
        storage_gb += sum(x.size_in_gbs for x in all_pages(
            block.list_volumes, compartment_id=compartment)
            if x.lifecycle_state != "TERMINATED")
        for domain in domains:
            storage_gb += sum(x.size_in_gbs for x in all_pages(
                block.list_boot_volumes, availability_domain=domain.name,
                compartment_id=compartment) if x.lifecycle_state != "TERMINATED")
    others = [x for x in instances if x.display_name != NAME]
    if others:
        raise RuntimeError("Other VMs exist; review the Always Free allowance manually before creating another")
    if storage_gb + (0 if instances else BOOT_GB) > 200:
        raise RuntimeError(f"Boot and block volumes would exceed 200 GB: {storage_gb} GB already allocated")
    return named(instances, NAME), storage_gb


def current_public_ip():
    with urlopen("https://checkip.amazonaws.com/", timeout=10) as response:
        return str(ipaddress.IPv4Address(response.read().decode().strip()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--plan", action="store_true", help="Check the account without creating resources")
    mode.add_argument("--apply", action="store_true", help="Create or reuse the Always Free VM")
    args = parser.parse_args()

    config = oci.config.from_file(str(Path.home() / ".oci" / "config"))
    oci.config.validate_config(config)
    tenancy = config["tenancy"]
    identity = oci.identity.IdentityClient(config)
    home = next((r.region_name for r in identity.list_region_subscriptions(tenancy).data
                 if r.is_home_region), None)
    if not home:
        raise RuntimeError("Oracle home region could not be identified")
    config["region"] = home
    identity = oci.identity.IdentityClient(config)
    compute = oci.core.ComputeClient(config)
    network = oci.core.VirtualNetworkClient(config)
    block = oci.core.BlockstorageClient(config)
    domains = all_pages(identity.list_availability_domains, tenancy)
    existing, storage_gb = check_allowance(identity, compute, block, tenancy, domains)
    images = all_pages(compute.list_images, tenancy, shape=SHAPE)
    candidates = [x for x in images if x.display_name.startswith("Canonical-Ubuntu-24.04-aarch64-")
                  and "Minimal" not in x.display_name]
    if not candidates:
        raise RuntimeError("No official Ubuntu 24.04 Arm image is available")
    image = max(candidates, key=lambda x: x.time_created)
    ssh_public = Path.home() / ".oci" / "wt-em-ssh.pub"
    if not ssh_public.is_file():
        raise RuntimeError(f"SSH public key missing: {ssh_public}")
    ssh_cidr = current_public_ip() + "/32"
    print(f"Home region: {home}; domains: {len(domains)}; current storage: {storage_gb} GB")
    print(f"VM: {SHAPE}, {OCPUS} OCPUs, {MEMORY_GB} GB RAM, {BOOT_GB} GB boot disk")
    print(f"Image: {image.display_name}; SSH limited to {ssh_cidr}")
    if args.plan:
        print("Plan only: no cloud resources created.")
        return

    m = oci.core.models
    vcn = named(all_pages(network.list_vcns, tenancy), NAME + "-vcn")
    if not vcn:
        vcn = network.create_vcn(m.CreateVcnDetails(
            compartment_id=tenancy, display_name=NAME + "-vcn",
            cidr_block="10.42.0.0/16", dns_label="wtemfree")).data
        vcn = wait_for(lambda: network.get_vcn(vcn.id), "AVAILABLE")
    print("VCN ready")

    gateway = named(all_pages(network.list_internet_gateways, tenancy, vcn_id=vcn.id), NAME + "-gateway")
    if not gateway:
        gateway = network.create_internet_gateway(m.CreateInternetGatewayDetails(
            compartment_id=tenancy, vcn_id=vcn.id, display_name=NAME + "-gateway",
            is_enabled=True)).data
        gateway = wait_for(lambda: network.get_internet_gateway(gateway.id), "AVAILABLE")
    route = named(all_pages(network.list_route_tables, tenancy, vcn_id=vcn.id), NAME + "-route")
    if not route:
        route = network.create_route_table(m.CreateRouteTableDetails(
            compartment_id=tenancy, vcn_id=vcn.id, display_name=NAME + "-route",
            route_rules=[m.RouteRule(destination="0.0.0.0/0", destination_type="CIDR_BLOCK",
                                     network_entity_id=gateway.id)])).data
        route = wait_for(lambda: network.get_route_table(route.id), "AVAILABLE")

    rules = [m.IngressSecurityRule(protocol="6", source=source,
             tcp_options=m.TcpOptions(destination_port_range=m.PortRange(min=port, max=port)))
             for port, source in ((22, ssh_cidr), (80, "0.0.0.0/0"), (443, "0.0.0.0/0"))]
    security = named(all_pages(network.list_security_lists, tenancy, vcn_id=vcn.id), NAME + "-security")
    if not security:
        security = network.create_security_list(m.CreateSecurityListDetails(
            compartment_id=tenancy, vcn_id=vcn.id, display_name=NAME + "-security",
            ingress_security_rules=rules,
            egress_security_rules=[m.EgressSecurityRule(protocol="all", destination="0.0.0.0/0")])).data
        security = wait_for(lambda: network.get_security_list(security.id), "AVAILABLE")
    subnet = named(all_pages(network.list_subnets, tenancy, vcn_id=vcn.id), NAME + "-subnet")
    if not subnet:
        subnet = network.create_subnet(m.CreateSubnetDetails(
            compartment_id=tenancy, vcn_id=vcn.id, display_name=NAME + "-subnet",
            cidr_block="10.42.1.0/24", dns_label="app", route_table_id=route.id,
            security_list_ids=[security.id], prohibit_public_ip_on_vnic=False)).data
        subnet = wait_for(lambda: network.get_subnet(subnet.id), "AVAILABLE")
    print("Network ready")

    if not existing:
        for domain in domains:
            shapes = all_pages(compute.list_shapes, tenancy, availability_domain=domain.name)
            if not any(x.shape == SHAPE for x in shapes):
                continue
            try:
                existing = compute.launch_instance(m.LaunchInstanceDetails(
                    compartment_id=tenancy, availability_domain=domain.name,
                    display_name=NAME, shape=SHAPE,
                    shape_config=m.LaunchInstanceShapeConfigDetails(
                        ocpus=OCPUS, memory_in_gbs=MEMORY_GB),
                    source_details=m.InstanceSourceViaImageDetails(
                        source_type="image", image_id=image.id,
                        boot_volume_size_in_gbs=BOOT_GB),
                    create_vnic_details=m.CreateVnicDetails(
                        subnet_id=subnet.id, assign_public_ip=True),
                    metadata={"ssh_authorized_keys": ssh_public.read_text(encoding="utf-8").strip()}
                )).data
                break
            except oci.exceptions.ServiceError as error:
                if error.code not in ("OutOfHostCapacity", "OutOfCapacity"):
                    raise
                print(f"No A1 capacity in {domain.name}; trying another domain")
        if not existing:
            raise RuntimeError("No Always Free A1 capacity in any availability domain")
    existing = wait_for(lambda: compute.get_instance(existing.id), "RUNNING", timeout=900)
    attachments = all_pages(compute.list_vnic_attachments, tenancy, instance_id=existing.id)
    if len(attachments) != 1:
        raise RuntimeError("Expected one VM network attachment")
    public_ip = network.get_vnic(attachments[0].vnic_id).data.public_ip
    if not public_ip:
        raise RuntimeError("The VM has no public IP")
    print(f"VM running: {public_ip}")
    print(f"Next: python deploy/oracle/deploy.py --host {public_ip} --key {Path.home() / '.oci' / 'wt-em-ssh'}")


if __name__ == "__main__":
    try:
        main()
    except (oci.exceptions.ServiceError, RuntimeError, TimeoutError, OSError) as error:
        print(f"Provisioning stopped: {error}", file=sys.stderr)
        raise SystemExit(1)
