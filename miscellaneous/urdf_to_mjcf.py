# urdf to mujoco conversion
import mujoco
model = mujoco.MjModel.from_xml_path("/home/ros/rebird_ws/src/rebird_description/urdf/rebird_3.urdf")
mujoco.mj_saveLastXML("rebird_3_proper.xml", model) 