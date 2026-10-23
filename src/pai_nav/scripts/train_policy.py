"""Reproducible imitation learning: synthetic sensor records -> 5-NN model.
No ROS / GPU / third-party Python libraries required.
"""
import json
import random
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from pai_nav.policy import teacher,KNNPolicy

rng=random.Random(20261022)

def point():
    # Oversample nearer obstacles so training covers meaningful evasive maneuvers.
    front=rng.choice([rng.uniform(0,0.3),rng.uniform(0.3,0.55),rng.uniform(0.55,1)])
    return [round(front,4),round(rng.random(),4),round(rng.random(),4),round(rng.uniform(-1,1),4)]

train=[]
test=[]
for i in range(1800):
    x=point()
    row=x+[teacher(*x)]
    (train if i<1500 else test).append(row)
policy=KNNPolicy(train,k=5)
acc=sum(policy.predict(x[:4])==x[4] for x in test)/len(test)
model={'description':'Imitation learning from synthetic expert obstacle-avoidance decisions',
       'seed':20261022,'k':5,'test_accuracy':round(acc,4),'train_count':len(train),
       'test_count':len(test),'features':['front/2m','left/2m','right/2m','goal_angle/pi'],
       'classes':['forward','turn_left','turn_right'],'samples':train}
path=Path(__file__).resolve().parents[1]/'models'/'knn_model.json'
path.write_text(json.dumps(model,ensure_ascii=False,separators=(',',':')),encoding='utf-8')
print(f'Model saved: {path}; held-out synthetic-data accuracy={acc:.2%} (not physical navigation success)')
