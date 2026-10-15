"""Fast website session orchestration over the unchanged recovered model.

Profiles are validated at construction/checkpoint boundaries. Within a flight,
share fixed properties and clone mutable state before each transactional step.
This adapter is for the website's owned, read-only-property callbacks; external
research callers continue using the reference sessions by default.
"""
from copy import copy, deepcopy
import math
from frame_session import FrameSession, STEP
from interaction_session import InteractionSession, SCOPE
from flight_session import finite
from flight_limits import evaluate
from tracking_timeout import advance as advance_tracking_timeout, threshold
from kernels import f32, add, sub
from proximity_request import advance_clock, build, prepare
from proximity_motion import sample_segment
from proximity_geometry import point_candidate
from trace_events import limit_pending, consume_no_contacts


class FastFrameSession(FrameSession):
    def advance(self,now,target_provider,*,illumination_provider=None,flight_step=None):
        """One supplied entity visit; provider(old_body, next_time) supplies targets.

        The native no-helper scheduler uses fixed 1/48 body intervals, caps its
        requested target at old time + 0.5, and permits iteration_limit - 1
        updates. It can overshoot the requested time. Expiry uses supplied now
        and the post-update position, not an interpolated threshold crossing.
        The engaged/untracked timer is updated after each guidance evaluation;
        its destruction callback marks pending expiry without interrupting the
        current catch-up loop. Remote-owner suppression and nonnegative
        guidance-event fractions remain outside this pure-air interface.
        """
        now=f32(now)
        if not math.isfinite(now):raise ValueError('Finite caller time required')
        if self.expiry_flag:
            return dict(status='expiry_dispatch_due',now=now,updates=[],state=deepcopy(self.flight.state),
                position=self.flight.state['body']['position'][:])
        # Commit only after all supplied inputs and independent steps succeed.
        working=copy(self.flight);working.state=deepcopy(self.flight.state);start=working.state['body']['time']
        lost_clock=self.lost_tracking_clock;destroy=False
        target=min(add(start,.5),now);updates=[];counter=0
        while working.state['body']['time']<target:
            counter+=1
            if counter>=self.iteration_limit:break
            old=deepcopy(working.state['body']);next_time=add(old['time'],STEP)
            targets=target_provider(old,next_time)
            result=working.step(targets,STEP,illumination_provider=illumination_provider,flight_step=flight_step)
            manager=working.state['guidance']['manager']
            loss=advance_tracking_timeout(lost_clock,threshold(working.properties['rocket']),STEP,
                engaged=manager['engaged'],tracking=manager['tracking'])
            lost_clock=loss['clock'];destroy=destroy or loss['destroy']
            updates.append(result)
        expiry=evaluate(**self.limits,position=working.state['body']['position'],now=now,initial_flag=self.expiry_flag or destroy)
        self.flight=working;self.expiry_flag=expiry['expiry_flag'];self.lost_tracking_clock=lost_clock
        return dict(status='expiry_pending' if self.expiry_flag else 'active',now=now,target_time=target,
            advanced_time=sub(working.state['body']['time'],start),updates=updates,expiry=expiry,state=deepcopy(working.state))


class FastInteractionSession(InteractionSession):
    def __init__(self, document):
        super().__init__(document)
        frame = FastFrameSession.__new__(FastFrameSession)
        frame.__dict__.update(self.frame.__dict__)
        self.frame = frame
        # FlightSession has already validated this exact layout against the
        # profile. It is a tuple of frozen dataclasses, private to this owned
        # flight; checkpoints still serialize the original authored properties.
        frame.flight.properties['_motor_properties'] = frame.flight.properties['guidance']['motors']

    def advance(self,now,frame_dt,target_provider,*,collision_time,point_history=None,
                illumination_provider=None,flight_step=None,surface=lambda endpoint:0.):
        """Advance one visit and consume one optional moving-point candidate.

        target_provider supplies guidance observations; point_history supplies
        collision history. Both must describe the intended target. collision_time
        is the explicit scene/consumer base clock, separate from body time.
        State commits only if propagation and every scene callback succeed.
        """
        if self.event is not None:
            return dict(status='event_dispatch_due',event=deepcopy(self.event),updates=[],scope=SCOPE)
        if self.frame.expiry_flag:
            return dict(status='expiry_dispatch_due',event=None,updates=[],scope=SCOPE)
        now,frame_dt,collision_time=map(f32,(now,frame_dt,collision_time))
        if not all(map(math.isfinite,(now,frame_dt,collision_time))) or frame_dt<0.:
            raise ValueError('Finite caller clocks and nonnegative frame_dt required')
        work=copy(self.frame);old=deepcopy(work.flight.state['body'])
        result=work.advance(now,target_provider,illumination_provider=illumination_provider,flight_step=flight_step)
        rocket=work.flight.serialized_properties['rocket'];has_fuse=rocket.get('hasProximityFuse',False)
        clock=advance_clock(self.proximity['clock'],frame_dt,has_fuse,not result['expiry']['returned'])
        body=result['state']['body'];manager=result['state']['guidance']['manager']
        descriptor=build(start=old['position'],end=body['position'],origin=self.proximity['origin'],elapsed=clock,
            enabled=self.proximity['enabled'],has_fuse=has_fuse,detect_shells=self.props['detectShells'],mode=False,
            prior_body_time=old['time'],body_time=body['time'],guidance_distance=manager['distance'],
            guidance_flag=self.proximity['guidance_flag'])
        descriptor=limit_pending(descriptor,mode=self.mode,delay=self.delay,velocity=body['velocity'])
        preparation=motion=None;candidate=-1.;consumed=None
        if descriptor['queued']:
            preparation=prepare(descriptor,self.props,surface)
            descriptor=preparation['descriptor']
            if preparation['air_enabled'] and point_history is not None:
                end=preparation['endpoint'];lower,upper=[add(collision_time,x) for x in self.offsets]
                motion=sample_segment(descriptor['start'],end,
                    (descriptor['prior_body_time'],descriptor['body_time']),lower,upper,
                    point_history.position,point_history.attitude)
                finite(motion,'point_history')
                candidate=point_candidate(point=motion['matrix'][9:],start=descriptor['start'],end=motion['end'],
                    origin=descriptor['origin'],elapsed=clock,timeout=self.props['timeOut'],
                    arm_distance=self.props['armDistance'],radius=self.props['radius'],scale=descriptor['length'])
                descriptor['candidate_results'][1]=candidate
            consumed=consume_no_contacts(descriptor,mode=self.mode,delay=self.delay,velocity=body['velocity'],
                                          base_time=collision_time,offsets=self.offsets)
        self.frame=work;self.proximity['clock']=clock
        if consumed is not None:self.mode=consumed['mode'];self.event=consumed['event']
        return dict(status='event_pending' if self.event is not None else result['status'],
            frame=result,descriptor=descriptor,preparation=preparation,motion=motion,candidate=candidate,
            consumer=consumed,event=deepcopy(self.event),scope=SCOPE)
