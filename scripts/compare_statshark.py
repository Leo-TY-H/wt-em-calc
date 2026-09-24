"""Compare saved Statshark numerical data to the local scalar reconstruction."""
import json,math
from pathlib import Path
from polar_model import make_polar,calc_cl,calc_cd
from thrust_table import steady_thrust

ROOT=Path(__file__).resolve().parent.parent

def main():
    data=json.loads((ROOT/'references/statshark/graphs.json').read_text());rows=[]
    for aircraft,charts in data.items():
        fm=json.loads((ROOT/f'references/fm-2.59.0.13/{aircraft}.blkx').read_text())
        for key,component in [('wing','WingPlane'),('fuselage','FuselagePlane'),('hStab','HorStabPlane'),('vStab','VerStabPlane')]:
            plane=fm['Aerodynamics'][component];props=plane.get('Polar',plane.get('FlapsPolar0'))
            area=sum(plane['Areas'][k] for k in ['LeftIn','LeftMid','LeftOut','RightIn','RightMid','RightOut']) if component=='WingPlane' else sum(plane['Areas'].values())
            for field,fn in [('liftCoefficient',calc_cl),('dragCoefficient',calc_cd)]:
                errors=[];worst=None
                for curve in charts['clCd']['clcd']:
                    p=make_polar(props,plane['Span'],area,curve['machNumber'])
                    for alpha,reference in zip(charts['clCd']['alpha'],curve[key][field]):
                        predicted=fn(p,alpha);error=abs(predicted-reference);errors.append(error)
                        if worst is None or error>worst['absolute_error']:worst=dict(mach=curve['machNumber'],alpha=alpha,predicted=predicted,statshark=reference,absolute_error=error)
                rows.append(dict(aircraft=aircraft,component=component,quantity=field,points=len(errors),max_absolute_error=max(errors),rms_error=math.sqrt(sum(x*x for x in errors)/len(errors)),worst=worst))
        for filename,afterburner in [('graphs-dry.json',False),('graphs-wet-altitudes.json',True)]:
            thrust_data=json.loads((ROOT/'references/statshark'/filename).read_text())[aircraft]['thrust']
            for curve in thrust_data['altitudeData']:
                errors=[];altitude=curve['altitude'];values=curve['data']
                for speed,thrust in zip(values['vel'],values['thrust']):errors.append(abs(steady_thrust(fm['EngineType0']['Main'],altitude,speed,afterburner)-thrust))
                rows.append(dict(aircraft=aircraft,quantity='thrust_kgf',altitude_m=altitude,afterburner=afterburner,points=len(errors),max_absolute_error=max(errors),rms_error=math.sqrt(sum(x*x for x in errors)/len(errors))))
    report=dict(source='https://statshark.net/fm',request_file='references/statshark/request.json',response_files=['references/statshark/graphs.json','references/statshark/graphs-dry.json','references/statshark/graphs-wet-altitudes.json'],comparisons=rows,limitations='Statshark is a secondary reference. Component curves are not whole-aircraft force validation. Response alpha values are rounded to 0.1 degree and can repeat despite 0.05 requested step. Mach preparation here uses double-precision normalized cubic evaluation; small coefficient differences are not assigned a cause without further verification.')
    (ROOT/'analysis/statshark-comparison.json').write_text(json.dumps(report,indent=2))
    for r in rows:print(r['aircraft'],r.get('component',''),r['quantity'],r['points'],'max error',r['max_absolute_error'])
if __name__=='__main__':main()
