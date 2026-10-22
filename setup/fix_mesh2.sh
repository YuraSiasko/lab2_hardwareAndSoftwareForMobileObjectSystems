#!/usr/bin/env bash
exec > >(tee "$(dirname "$0")/fix_mesh2.txt") 2>&1
I=/opt/ardu_ws/install/ardupilot_gazebo/share/ardupilot_gazebo
M=$I/models/iris_with_standoffs/model.sdf
[ -f $M.orig ] || cp $M $M.orig
sed -i "s#package://ardupilot_gazebo/models/iris_with_standoffs/meshes/#file://$I/models/iris_with_standoffs/meshes/#g" $M
grep -n "<uri>" $M | head
echo "== other places referencing package:// in models used"; grep -rl "package://" /opt/ardu_ws/install/*/share/*/models /opt/ardu_ws/install/*/share/*/worlds 2>/dev/null | head
echo MESH_FIX_DONE
