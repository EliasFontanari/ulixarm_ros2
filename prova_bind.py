import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

import damiao_motor_control as dmc

kp_list = [100.0, 100.0, 100.0, 80.0, 80.0, 80.0, 50.0]
kd_list = [0.5,0.5,0.5,0.3,0.3,0.3,0.1]

def load_joint_names(urdf_path: Path) -> list[str]:
	root = ET.parse(urdf_path).getroot()
	joint_list = [joint.get("name") for joint in root.findall("joint") if joint.get("name") and joint.get("type") == "revolute"]
	joint_list.append("gripper_joint")
	return joint_list


# def write_motor(joints_pos,joints_vel,torques,kps,kds):
#     for i, joint_name in enumerate(joint_names):
#         status = mc.control_mit(joint_name, joints_pos[i], joints_vel[i], torques[i], kps[i], kds[i])
#         if status < 0:
#             print(f"Errore durante il controllo del motore {joint_name}: {status}")
#             exit(1)
            
#     # ricevere dati motore
#     for _ in range(len(joint_names)):
#         mc.receive_motor_data()
        
print("Inizio prova bind")

urdf_path =	"src/ulixarm_description/src/ulixarm_description/urdf/robot.urdf"
joint_names = load_joint_names(urdf_path)
print("Joint names:")
for name in joint_names:
	print(f"- {name}")
print(f"Totale joint: {len(joint_names)}")


# on_init/configure equivalent
print("Creazione oggetto MotorControl")
mc = dmc.MotorControl()
print("Creazione oggetto MotorControl riuscita")
status = mc.init("/dev/ttyACM0", 921600, 50)
if status < 0:
    print(f"Errore durante l'inizializzazione di MotorControl: {status}")
    exit(1)
else:
    print("Inizializzazione oggetto MotorControl riuscita")

print("Inizializzazione motori")

# Initialize motors
mc.add_motor(joint_names[0], dmc.DMMotorType.DM4340, 0x01, 0x11)
mc.add_motor(joint_names[1], dmc.DMMotorType.DM4340, 0x02, 0x12)
mc.add_motor(joint_names[2], dmc.DMMotorType.DM4340, 0x03, 0x13)
mc.add_motor(joint_names[3], dmc.DMMotorType.DM4310, 0x04, 0x14)
mc.add_motor(joint_names[4], dmc.DMMotorType.DM4310, 0x05, 0x15)
mc.add_motor(joint_names[5], dmc.DMMotorType.DM4310, 0x06, 0x16)
mc.add_motor(joint_names[6], dmc.DMMotorType.DMJ3507, 0x07, 0x17)

print("Motori inizializzati")

print("Abilitazione dei motori")
status = mc.enable_motor_all()
if status < 0:
    print(f"Errore durante l'abilitazione dei motori: {status}")
    exit(1)
else:    print("Abilitazione dei motori riuscita")

print("Refreshing dei motori")
status = mc.refresh_motor_status_all()

print("Lettura motori per inizializzare stato interno")
for i in range(len(joint_names)):
    print(i)
    status = mc.receive_motor_data()
    if status < 0:
        print(f"Errore durante la ricezione dei dati del motore {joint_names[i]}: {status}")
        exit(1)

pos = 0.
print(f'Posizioni iniziali dei motori: [{mc.get_position(joint_names[0])}, \
                                        {mc.get_position(joint_names[1])}, \
                                        {mc.get_position(joint_names[2])}, \
                                        {mc.get_position(joint_names[3])}, \
                                        {mc.get_position(joint_names[4])}, \
                                        {mc.get_position(joint_names[5])}, \
                                        {mc.get_position(joint_names[6])}]')

if status < 0:
    print(f"Errore durante il refresh dei motori: {status}")
    exit(1)
else:
    print("Refresh dei motori riuscito")
    


