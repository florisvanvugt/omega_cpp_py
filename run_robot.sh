#! /bin/sh

sudo ./Robot &

sleep .5

# Make sure that other processes can read the shared memory
sudo chmod a+rw /tmp/sharedmemory.bin

exit 0
