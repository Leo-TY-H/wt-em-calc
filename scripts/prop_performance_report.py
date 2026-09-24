"""Render the propulsion search and native replay, without aircraft EM claims."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    root=Path(__file__).resolve().parents[1];data=json.loads((root/'analysis/prop-performance.json').read_text())
    validation=json.loads((root/'analysis/prop-performance-validation.json').read_text())
    assert validation['status']=='PASS' and len(data['rows'])==18
    assert all(r['rpm_limit_respected'] for r in validation['rows'])
    replay={(r['aircraft'],r['height_m'],r['speed_mps'],r['mode']):r for r in validation['rows']}
    names={'yak-3':'Yak-3','bf-109f-4':'Bf 109 F-4'}
    fig,axes=plt.subplots(3,2,figsize=(10,10),sharex=True,sharey='row')
    for col,n in enumerate(names):
        for row,h in enumerate([0.,3000.,6000.]):
            rs=[r for r in data['rows'] if r['aircraft']==n and r['height_m']==h];ax=axes[row,col]
            for mode,color,label,style in [('best','#156f9a','Best sampled command','-o'),('automatic_prop_with_best_gear','#aa633b','Automatic prop target','--s')]:
                ax.plot([r['speed_mps']*3.6 for r in rs],[replay[n,h,r['speed_mps'],mode]['thrust_mean_N']/1000 for r in rs],style,color=color,label=label)
            ax.set_title(names[n]+f' · {h/1000:g} km');ax.grid(alpha=.2);ax.set_ylabel('Axial thrust (kN)');ax.set_ylim(bottom=0)
            if row==2:ax.set_xlabel('Prescribed forward airspeed (km/h)')
            if row==0:ax.legend(fontsize=8)
    for row,h in enumerate([0.,3000.,6000.]):
        maximum=max(v['thrust_mean_N']/1000 for k,v in replay.items() if k[1]==h)
        axes[row,0].set_ylim(0,maximum*1.08)
    fig.suptitle('Propulsion equilibrium: Yak-3 and Bf 109 F-4',fontsize=15)
    fig.text(.5,.025,'Native-replayed samples; lines only connect samples. Full power/WEP, ideal mixture, free air, closed radiators.\nBest legal gear in both series; RPM ≤ configured maximum allowed. Aircraft lift, drag and trim are not solved.',ha='center',fontsize=9)
    fig.tight_layout(rect=[0,.065,1,.97]);out=root/'outputs/propeller-research';out.mkdir(parents=True,exist_ok=True)
    for ext in ['png','svg','pdf']:fig.savefig(out/('propulsion-comparison.'+ext),dpi=180)
    lines=['# Propeller operating-point research','','The search and exact native replay support speed/altitude-dependent propeller settings. Maximum RPM is not uniformly the best-thrust choice. All values below are conditional propulsion results at prescribed forward airspeed, with no aircraft lift/drag/trim solution.','','![Sampled thrust comparison](../outputs/propeller-research/propulsion-comparison.png)','','## Assumptions and method','','Ordinary Yak-3/German Bf109F4, pinned FM2.59.0.13 and the saved executable hash. Yak uses throttle1 and its best selectable compressor stage; Bf uses throttle1.1 with WEP. Mixture is selected on the delivered .005 command grid within the full-power interval. Water/oil radiators are closed. Fuel/health are fixed. Both comparison series use the best legal gear: the dashed series is an automatic **propeller RPM target** comparator, not a reconstruction of the entire automatic engine-management policy.','','For each of18 speed/altitude conditions, search all256 delivered propeller commands and each legal gear, using several starting guesses for torque/inflow equilibrium. Automatic-prop roots are also searched from multiple guesses. Reject torque/inflow residuals above3e-5 in normalized units, governor-infeasible roots, omega<150rad/s and RPM above RPMMaxAllowed (Yak2800/Bf3000). Yak manual RPM-governor targets themselves cannot exceed2700. Bf manual pitch can select a higher engine equilibrium speed than the automatic2700RPM WEP target. Neither closed radiators nor frozen damage is used to justify exceeding these configured RPM limits.','','The search assumes unit healthy mechanical multiplier. Every selected candidate and automatic comparator is then replayed with the complete original propulsion owner and an independent Python state:720 steps at48Hz; the final240 samples supply the averages below. This preserves mechanical, governor, regulator and wake histories. All720 samples respect configured RPM limits. The scalar imported math and loader/raycast boundaries are documented in the main report.','','## Sampled results','','Thrust is mean native replay thrust, rounded to the nearest newton. RPM is the search equilibrium, rounded to the nearest RPM. Gear is the one-based displayed stage number. Control is the raw delivered prop command, not blade pitch percentage shared between aircraft.','','| Aircraft | Altitude m | Speed km/h | Best thrust N | Auto target N | Difference | Best RPM | Raw command | Gear |','|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for r in data['rows']:
        n,h,u=r['aircraft'],r['height_m'],r['speed_mps'];w=r['best'];b=replay[n,h,u,'best']['thrust_mean_N'];a=replay[n,h,u,'automatic_prop_with_best_gear']['thrust_mean_N']
        lines.append(f"| {names[n]} | {h:.0f} | {u*3.6:.0f} | {b:.0f} | {a:.0f} | {100*(b/a-1):+.2f}% | {w['rpm']:.0f} | {w['command']:.5f} | {w['gear']+1} |")
    lines+=['','The large low-speed Bf differences are local native-model outcomes: the automatic2700RPM equilibrium can lie on a poorly performing blade-loading branch, while a permitted manual-pitch setting reaches a higher-RPM, higher-thrust equilibrium. They are not evidence of the same increase in climb rate or a measured in-game advantage. Both candidate states were individually replayed against original code.','','These are best results among converged sampled roots, not a proof that every root or control history was found. Distinct equilibria exist for some commands. A later aircraft-performance calculation must jointly solve lift/drag, torque/gyro trim, control limits, angle-dependent prop inflow and mass/loadout. The Bf now has records produced from installed collision geometry; actual ammunition/loadout and live pose remain explicit inputs.','','## Evidence','','- [Search data and residuals](prop-performance.json)','- [25,920 exact native/independent owner steps](prop-performance-validation.json)','- [1,440 complete aircraft replay steps](prop-coupled-airborne-validation.json)','- [Full reconstruction, equations and boundaries](propeller-aircraft-findings.md)','','Exports: [SVG](../outputs/propeller-research/propulsion-comparison.svg), [PDF](../outputs/propeller-research/propulsion-comparison.pdf). Reproduce with `prop_performance.py`, `verify_prop_performance.py`, then `prop_performance_report.py`.']
    (root/'analysis/prop-performance-findings.md').write_text('\n'.join(lines)+'\n')
    print('Saved performance notes and PNG/SVG/PDF')
if __name__=='__main__':main()
