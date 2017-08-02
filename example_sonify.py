


import robot
import time



import pyaudio
import math
import numpy as np



class PitchPlayer:

    # How big chunks should be written at one time
    #CHUNKSIZE = 4096
    #CHUNKSIZE = 88200

    #AMPLITUDE = 127
    #OFFSET = 128

    AMPLITUDE = .25

    # Sampling rate
    RATE = 44100
    #RATE = 22050



    def __init__(self):
        self.p = pyaudio.PyAudio()
        self.stop_now = True

        # These control the playing of the sound. Each wave corresponds to an entry in all three lists
        self.pitches       = []  # the pitch (in Hz)
        self.factors       = []  # the factor: phase increment as a function of frame
        self.phase_offsets = []  # the current phase (when playing is active)





    def get_chunk(self,frame_count):
        # The wave associated with the scale note
        wav = np.zeros(frame_count)

        for (offset,factor) in zip(self.phase_offsets,self.factors):
            wav += self.AMPLITUDE*np.sin([ offset + factor*x for x in range(frame_count) ])

        # Update the phases: advance by factor*time for each
        self.phase_offsets = [ (offs + frame_count*fact)%(2*math.pi) for offs,fact in zip(self.phase_offsets,self.factors) ]

        return wav.astype(np.float32)



    def callback(self,in_data, frame_count, time_info, status):
        chunk = self.get_chunk(frame_count)
        #print chunk
        #self.allplayed.append(chunk)
        #print(chunk)
        if self.stop_now:
            return (chunk, pyaudio.paComplete)
        else:
            return (chunk, pyaudio.paContinue)



    def setpitch(self,pitch):
        """
        Update the frequency we should be playing (in Hz).
        Also, if base is not None then we also play the base note
        at the given frequency.
        """
        self.pitches = pitch
        #print ("Currently playing: %s"%(" ".join([ "%.02f"%p for p in self.pitches ])))
        # Compute the factors (phase increase per unit of time
        self.factors  = [ float(pitch) * (2 * math.pi) / self.RATE for pitch in self.pitches ]
        if len(self.phase_offsets)!=len(self.factors):
            self.reset_phases()
        # TODO: instead of resetting phases we could do something smarter, namely
        # take the existing phases of oscillators we are keeping, and only
        # resetting phases of oscillators that are "new".




    def play(self):
        """ Starts playing, assuming that a correct pitch has been set. """
        #data = ''.join([chr(int(math.sin(x/((RATE/pitch)/math.pi))*127+128)) for x in xrange(RATE)])
        if self.stop_now: # only start a new stream if we aren't already playing
            self.stop_now = False
            self.reset_phases()
            self.stream = self.p.open(format          = pyaudio.paFloat32,
                                      channels        = 1,
                                      rate            = self.RATE,
                                      output          = True,
                                      stream_callback = self.callback
                                  )


    def reset_phases(self):
        self.phase_offsets = [ 0. for _ in self.pitches ]



    def stop(self):
        #self.stream.stop_stream()
        self.stop_now = True
        #self.stream.close()
        #pickle.dump(self.allplayed,open('chunk.pickle','wb'))


    def close(self):
        self.p.terminate()



robot.launch()
robot.init()

player = PitchPlayer()
player.setpitch([200,1100,1800])

t0 = time.time()
t1 = time.time()
player.play()

x,y,z=0,0,0

while t1-t0<10:

    x = robot.rshm('x')#, robot.rshm('y'), robot.rshm('z')
    time.sleep(0.001) # INTERESTING: this has to be here because otherwise you get interference in the sound ?!
    y = robot.rshm('y')
    #time.sleep(0.001)
    z = robot.rshm('z')
    #time.sleep(0.001)

    #z=0
    f1 = 200 + 1000*x
    f2 = 1100 + 1500*y
    f3 = 1800 + 2000*z
    player.setpitch([f1,f2,f3])
    t1 = time.time()
    time.sleep(0.05)

player.stop()

robot.unload()
