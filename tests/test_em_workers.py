"""Worker selection respects the CPUs a deployment can actually use."""
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import em_workers


class WorkerTests(unittest.TestCase):
    def budget(self,files,affinity=8):
        def read(path,*args,**kwargs):
            if str(path).replace('\\','/') not in files:raise FileNotFoundError(path)
            return files[str(path).replace('\\','/')]
        with patch.object(os,'cpu_count',return_value=16), \
             patch.object(os,'sched_getaffinity',return_value=set(range(affinity)),create=True), \
             patch.object(Path,'read_text',read):
            return em_workers.cpu_budget()

    def test_affinity_and_quota(self):
        self.assertEqual(self.budget({},2),2)
        self.assertEqual(self.budget({'/sys/fs/cgroup/cpu.max':'200000 100000'}),2)
        self.assertEqual(self.budget({'/sys/fs/cgroup/cpu.max':'50000 100000'}),1)
        self.assertEqual(self.budget({'/sys/fs/cgroup/cpu.max':'max 100000'},2),2)

    def test_nested_and_legacy_quotas(self):
        self.assertEqual(self.budget({'/proc/self/cgroup':'0::/parent/child',
            '/sys/fs/cgroup/parent/cpu.max':'100000 100000'}),1)
        self.assertEqual(self.budget({'/sys/fs/cgroup/cpu/cpu.cfs_quota_us':'200000',
            '/sys/fs/cgroup/cpu/cpu.cfs_period_us':'100000'}),2)
        self.assertEqual(self.budget({'/proc/self/cgroup':'2:cpu,cpuacct:/parent/child',
            '/sys/fs/cgroup/cpu,cpuacct/parent/cpu.cfs_quota_us':'100000',
            '/sys/fs/cgroup/cpu,cpuacct/parent/cpu.cfs_period_us':'100000'}),1)

    def test_explicit_count_uses_both_cpus_but_not_more(self):
        with patch.object(em_workers,'cpu_budget',return_value=2),patch.dict(os.environ,{},clear=True):
            self.assertEqual(em_workers.worker_count(),1)
            for request,expected in [('1',1),('2',2),('12',2)]:
                os.environ['WT_EM_WORKERS']=request
                self.assertEqual(em_workers.worker_count(),expected)


if __name__=='__main__':unittest.main()
