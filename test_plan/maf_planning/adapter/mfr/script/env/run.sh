
on -C5 -p20 ./bin/mfrlaunch file -m mfrrpc://${MFR_IP}:11300 -c resource/launch/sbp_mfr_node.yaml &
on -C6 -p20 ./bin/mfrlaunch file -m mfrrpc://${MFR_IP}:11300 -c resource/launch/planning_mfr_node_qnx_test.yaml
#on -p20 ./bin/mfrlaunch file -m mfrrpc://${MFR_IP}:11300 -c resource/launch/planning_mfr_node.yaml
