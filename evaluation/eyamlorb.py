#!/usr/bin/env python for orb
import sys
import yaml
import ruamel.yaml
import ruamel.yaml.util
from pathlib import Path

#https://stackoverflow.com/questions/20364396/how-to-delete-the-first-line-of-a-text-file-using-python remove first line
#https://stackoverflow.com/questions/5914627/prepend-line-to-beginning-of-a-file
def line_prepender(filename, line):
    with open(filename, 'r+') as f:
        content = f.read()
        f.seek(0, 0)
        f.write(line.rstrip('\r\n') + '\n' + content)

#https://stackoverflow.com/questions/40762382/changing-a-value-in-a-yaml-file-using-python
file_name="/home/meow/catkin_ws/src/orb/orb_slam2/config/RealSenseD435RGBD.yaml"
with open(file_name, 'r') as conf:
    lines = conf.readlines()[1:]
    with open ("/home/meow/catkin_ws/src/orb/orb_slam2/config/test.yaml", 'w+') as savefile:
        for i in lines:
            savefile.write(i) 
with open ("/home/meow/catkin_ws/src/orb/orb_slam2/config/test.yaml", 'r') as conf:
    result, indent, block_seq_indent = ruamel.yaml.util.load_yaml_guess_indent(
    conf, preserve_quotes=True)

#results['nas']['mount_dirs'][0] = "haha"
#https://stackabuse.com/command-line-arguments-in-python/
#1.2
scaleFactor=float(sys.argv[1])
#8
nLevels=int(sys.argv[2])
#20
iniThFast=int(sys.argv[3])
#7
minThFast=int(sys.argv[4])

print scaleFactor,nLevels,iniThFast,minThFast

result['ORBextractor.scaleFactor'] = scaleFactor
result['ORBextractor.nLevels'] = nLevels
result['ORBextractor.iniThFAST'] = iniThFast
result['ORBextractor.minThFAST'] = minThFast
with open(file_name, 'w') as conf:
  ruamel.yaml.round_trip_dump(result, conf, indent=indent,block_seq_indent=block_seq_indent)
line_prepender(file_name,"%YAML:1.0")
