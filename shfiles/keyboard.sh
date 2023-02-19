cd ~/XTDrone/sensing/slam/vio
python3 vins_transfer_vinsfusion.py iris 0 & sleep 3;
cd ~/XTDrone/communication
python3 multirotor_communication.py iris 0 & sleep 3;


cd ~/XTDrone/control/keyboard
python3 multirotor_keyboard_control.py iris 1 vel 

# wait;


