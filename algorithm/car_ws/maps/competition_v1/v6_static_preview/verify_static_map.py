#!/usr/bin/env python3
from pathlib import Path
import json
p=Path(__file__).resolve().parent
info=json.loads((p/'zones.json').read_text(encoding='utf-8'))
assert info['resolution']==0.01
assert info['grid_size']==[280,280]
assert len(info['yellow'])==4 and len(info['blue'])==2
assert len(info['topology_nodes'])==9 and len(info['topology_edges'])==12
maps={}
for name in ['competition_navigation_map','competition_keepout_mask','competition_rules_preview']:
    with (p/(name+'.pgm')).open('rb') as f:
        assert f.readline()==b'P5\n'
        assert f.readline().strip()==b'280 280'
        assert f.readline().strip()==b'255'
        data=f.read()
    assert len(data)==280*280
    assert set(data).issubset({0,205,254})
    maps[name]=data
    config=(p/(name+'.yaml')).read_text()
    assert 'resolution: 0.01' in config and 'origin: [-0.2, -0.2, 0.0]' in config
assert maps['competition_keepout_mask']==maps['competition_rules_preview']
# normalized image top-left row index; x and y refer to pixel centers in metric map coordinates
idx=lambda x,y:(279-int((y+.2)/.01))*280+int((x+.2)/.01)
nav=maps['competition_navigation_map']; mask=maps['competition_keepout_mask']
assert nav[idx(.75,.75)]==254 and mask[idx(.75,.75)]==0
assert nav[idx(1.2,1.2)]==254 and mask[idx(1.2,1.2)]==254
assert mask[idx(2.25,2.25)]==254
assert mask[idx(2.5,2.5)]==205
print('PASS: ROS YAML/PGM 280x280 1cm, origin -0.2m; maps verified; 9 nodes/12 edges')
