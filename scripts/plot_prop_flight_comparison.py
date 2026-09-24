"""Plot measured turn runs against matched-condition diagnostic equilibria."""
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_prop_flights import OUT


def main():
    rows=json.loads((OUT/'comparison.json').read_text())
    rows=rows[1:]+rows[:1]
    labels=['Yak · 22:15\nInstructor','Yak · 22:26\nInstructor',
            'Yak · 22:32:14\nNo Instructor','Yak · 22:32:26\nNo Instructor','Bf 109 · 22:38\nInstructor']
    y=np.arange(len(rows))
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(1,3,figsize=(13,5.4),sharey=True)
    for i,r in enumerate(rows):
        t=r['telemetry'];p=r['matched_rate_point'];m=t['means']
        axes[0].scatter(p['alpha_deg']-m['AoA, deg'],i,color='#246aa5',label='Balanced at measured rate' if i==0 else None)
        if t['instructor']:
            axes[0].scatter(r['boundary']['alpha_deg']-m['AoA, deg'],i,color='#bd4933',marker='x',s=65,
                            label='Instructor permission boundary' if i==0 else None)
        axes[1].scatter(100*(p['engine_force_n'][0]/9.80665/m['Thrust, kgf']-1),i,color='#246aa5')
        axes[2].scatter(p['ps_mps'],i,color='#246aa5',label='Model' if i==0 else None)
        axes[2].scatter(t['ps_fit_mps'],i,color='#222222',marker='s',label='Flight: linear energy trend' if i==0 else None)
        axes[2].scatter(t['ps_endpoint_mps'],i,color='#222222',marker='o',facecolors='none',label='Flight: endpoint energy change' if i==0 else None)
    for ax in axes:
        ax.axvline(0,color='#999999',lw=.8,ls='--');ax.grid(axis='x',alpha=.2);ax.set_ylim(4.6,-.6)
    axes[0].set_yticks(y,labels);axes[0].set_xlabel('Model minus recorded AoA (degrees)')
    axes[1].set_xlabel('Model minus recorded thrust (%)')
    axes[2].set_xlabel('Specific excess power, Ps (m/s)')
    axes[0].legend(loc='lower center',bbox_to_anchor=(.5,1.01),fontsize=8,frameon=False)
    axes[2].legend(loc='lower center',bbox_to_anchor=(.5,1.01),fontsize=8,frameon=False)
    fig.suptitle('Flight recordings compared with the current solver',y=.99,fontsize=13)
    fig.text(.02,.025,'Matched mean TAS, altitude, total mass and measured turn rate; current model turn sense, zero sideslip.\nBf: 30% fuel + measured 28.4 kg at nominal CG for this diagnostic. Thrust telemetry definition and ammo locations remain unverified.',fontsize=9)
    fig.tight_layout(rect=(0,.1,1,.90))
    fig.savefig(OUT/'comparison.png',dpi=170)
    fig.savefig(OUT/'comparison.svg')


if __name__=='__main__':main()
