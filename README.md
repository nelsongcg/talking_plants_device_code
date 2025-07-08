# talking_plants_device_code

How to run

arduino-cli compile \                           
  --fqbn esp32:esp32:esp32 \
  --board-options PartitionScheme=huge_app \
  -v \     
  sketch_v2 && \
arduino-cli upload \
  --fqbn esp32:esp32:esp32 \
  --port /dev/tty.wchusbserial110 \
  sketch_v2

