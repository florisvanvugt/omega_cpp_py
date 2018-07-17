#! /bin/sh

sudo ./Robot &

sleep 1

# Make sure that other processes can read the shared memory
sudo chmod a+rw /tmp/sharedinstrmemory.bin
sudo chmod a+rw /tmp/sharedlivememory.bin

exit 0
