#!/usr/bin/env python3
"""Plot one or more trajectory CSVs and print distance/time metrics."""
import argparse
import csv
import xml.etree.ElementTree as ET
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def read_csv(path):
    with open(path, newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f'CSV contains no data: {path}')
    fields = ['x_m','y_m','elapsed_s','cumulative_distance_m','distance_to_goal_m']
    for row in rows:
        for field in fields:
            row[field] = float(row[field])
    return rows

def add_obstacles(ax, world_path):
    """Overlay static world obstacle models in world XY coordinates."""
    root = ET.parse(world_path).getroot()
    count = 0
    for model in root.findall('.//world/model'):
        name = model.get('name', '')
        if not name.startswith('obstacle_'):
            continue
        pose = model.findtext('pose', default='0 0 0 0 0 0')
        px, py = map(float, pose.split()[:2])
        radius = None
        for element in model.findall('.//collision/geometry/cylinder/radius'):
            if element.text:
                radius = float(element.text)
                break
        if radius is None:
            for element in model.findall('.//visual/geometry/cylinder/radius'):
                if element.text:
                    radius = float(element.text)
                    break
        if radius is None:
            print(f'Warning: no cylinder radius for {name}; drawing a point only')
            ax.scatter([px], [py], color='sienna', marker='s', s=50)
        else:
            ax.add_patch(plt.Circle((px, py), radius, color='sienna', alpha=0.6))
        ax.annotate(name, (px, py), xytext=(6, 6), textcoords='offset points', fontsize=9)
        print(f'{name}: x={px:.3f} m, y={py:.3f} m, radius={radius}')
        count += 1
    if count == 0:
        print('Warning: no obstacle_* models found in SDF')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_files', nargs='+', help='One or more recorded CSV files')
    parser.add_argument('--output', default='~/pai_ws/results/trajectory.png')
    parser.add_argument('--goal-x',type=float,default=4.0)
    parser.add_argument('--goal-y',type=float,default=0.0)
    parser.add_argument('--goal-radius',type=float,default=0.35)
    parser.add_argument('--world', default='~/pai_ws/src/pai_nav/worlds/obstacle_course.sdf', help='SDF world to overlay obstacle positions')
    args = parser.parse_args()
    fig, ax = plt.subplots(figsize=(9,6))
    for path in args.csv_files:
        rows = read_csv(path)
        x=[r['x_m'] for r in rows]
        y=[r['y_m'] for r in rows]
        label=Path(path).stem
        ax.plot(x,y,linewidth=1.8,label=f'{label} ({rows[-1]["cumulative_distance_m"]:.2f} m)')
        ax.scatter([x[0]],[y[0]],marker='o',s=45)
        ax.scatter([x[-1]],[y[-1]],marker='x',s=65)
        arrived = next((r for r in rows if r['distance_to_goal_m'] < args.goal_radius), None)
        elapsed = arrived['elapsed_s'] if arrived else rows[-1]['elapsed_s']
        print(f'{label}: samples={len(rows)}, path={rows[-1]["cumulative_distance_m"]:.2f} m, '
              f'elapsed={elapsed:.2f} sim s, goal_reached={bool(arrived)}, '
              f'last_goal_distance={rows[-1]["distance_to_goal_m"]:.2f} m')
    add_obstacles(ax, Path(args.world).expanduser())
    ax.scatter([args.goal_x],[args.goal_y],marker='*',s=180,label='Goal (odometry frame)')
    ax.add_patch(plt.Circle((args.goal_x,args.goal_y),args.goal_radius,fill=False,linestyle='--',alpha=0.5))
    ax.set_aspect('equal',adjustable='datalim')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Robot trajectory with SDF obstacle locations')
    ax.grid(True,alpha=0.35)
    ax.legend()
    fig.tight_layout()
    output=Path(args.output).expanduser()
    output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=180)
    print(f'Plot saved: {output.resolve()}')

if __name__=='__main__':
    main()
