"""Replay until AEB actually brakes; then integrate speed along ego's own path.

This uses the prerecorded ego route as road guidance, not a target future state.
AEB replaces the original driver's longitudinal motion after intervention.
"""
import bisect
import math


class RecordedEgoPath:
    def __init__(self, reference):
        self.times = reference.times
        self.states = [next(s for s in reference.frames[t] if s['id']==reference.ego_id) for t in self.times]
        self.arc = [0.0]
        for a,b in zip(self.states,self.states[1:]):
            self.arc.append(self.arc[-1]+math.hypot(b['x']-a['x'],b['y']-a['y']))
        self.distance = None

    def arc_at_time(self,time):
        i = min(bisect.bisect_right(self.times,time),len(self.times)-1)
        if i==0: return 0.0
        u = (time-self.times[i-1])/(self.times[i]-self.times[i-1])
        return self.arc[i-1]+u*(self.arc[i]-self.arc[i-1])

    def advance(self,ego,acceleration,dt,time):
        if self.distance is None:
            self.distance = self.arc_at_time(time)
        moving = min(dt,ego['speed']/-acceleration) if acceleration<0 else dt
        self.distance += ego['speed']*moving+0.5*acceleration*moving*moving
        result = ego.copy()
        i = bisect.bisect_right(self.arc,self.distance)
        if i>=len(self.arc):
            end = self.states[-1]
            for k in ['z','h','p','r']: result[k]=end[k]
            result['x']=end['x']+(self.distance-self.arc[-1])*math.cos(end['h'])
            result['y']=end['y']+(self.distance-self.arc[-1])*math.sin(end['h'])
        else:
            i=max(1,i)
            a,b=self.states[i-1],self.states[i]
            u=(self.distance-self.arc[i-1])/max(1e-12,self.arc[i]-self.arc[i-1])
            for k in ['x','y','z','p','r']: result[k]=a[k]+u*(b[k]-a[k])
            dh=math.atan2(math.sin(b['h']-a['h']),math.cos(b['h']-a['h']))
            result['h']=a['h']+u*dh
        result['speed']=max(0.,ego['speed']+acceleration*dt)
        result['vx']=result['speed']*math.cos(result['h'])
        result['vy']=result['speed']*math.sin(result['h'])
        return result
