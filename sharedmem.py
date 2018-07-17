import os
import mmap
import struct
import yaml
import numpy as np


specification_file = 'shared_memory_specification.yaml'


def init_specifications(basedir):
    """
    Read the shared_memory_specification.txt file and parse it
    """
    memspecf = "%s/%s"%(basedir,specification_file)
    print("Reading memory specification from %s"%memspecf)

    with open(memspecf,'r') as f:
        i=0
        # This variable tells us where the variables are to be found
        global specifications # start building a memory specification variable
        specifications = [{},{}]
        # The order of the variables
        global variable_order # keep track of the order of the variables
        variable_order = [[],[]]
        address = 0 # this variable keeps track of the address in shared memory where we currenlty are
        for memsp in yaml.load_all(f):
                for varname,vartype in memsp.iteritems():   # Progress lign by lign in the shared_memory_specification
                    sz = struct.calcsize(vartype)           # equals 8 except for recording data (equals number x 8)
                    if (i==0) :
                        specifications[0][varname] = (vartype,address,sz)
                        specifications[1][varname] = (vartype,address,sz)
                        variable_order[0].append(varname)
                        variable_order[1].append(varname)
                    elif (i==1) :
                        specifications[1][varname] = (vartype,address,sz)
                        variable_order[1].append(varname)
                    else :
                        print ("ERROR: to many parts in the shared_memory_specification ")
                        exit()
                    address += sz   # keep our running count of the memory offset
                i+=1






def get_specifications():
    return specifications

def get_variable_order():
    return variable_order

#Open the shared memory
def init_shared_memory():
    """ Initialise shared memory, connect to shared memory of a C++ process """
    global shm
    shm = {}
    for (label, flags, flags2 ) in zip (['instr','live'], [mmap.PROT_READ | mmap.PROT_WRITE , mmap.PROT_READ ],[os.O_RDWR , os.O_RDONLY ] ):
        path='/tmp/shared'+label+'memory.bin'
        if not os.path.exists(path):
           print("ERROR, shared memory file does not exist. Is the robot process running?")
           exit()
        fp = os.open(path,flags2) # maybe RO for live?
        shm[label]= (mmap.mmap(fp, 0, mmap.MAP_SHARED, flags))






def find_variable(var,label):
    """ Find the memory and data type associated with the variable var """
    if (label=='instr') :
        if var in specifications[0] :
            return (specifications[0][var])
        else :
            print ("ERROR: unknown variable %s in instr_memory "%var)
            exit()
    elif (label=='live'):
        if var in specifications[1] :
            return (specifications[1][var])
        else :
            print ("ERROR: unknown variable %s in live_memory "%var)
            exit()
    print("ERROR: label has to be 'instr' or 'live' %s "%var)
    exit()




# Read in the shared live memory
def rshm(var,label='live'):
    """ Reads the shared memory value associated with the given variable,
    for example rshm('x','live') """

    if isinstance(var, list):   #True if var is a list
        return [rshm(v, label) for v in var]

    tp,offset,size = find_variable(var, label)
    memchunk = shm[label][offset:(offset+size)]

    # Interpret the raw binary as the correct data type
    if (len(tp) == 1):
        interpr, = struct.unpack(tp,memchunk)  # "interpr,"" to gather val instead of (val,)
    else:
        interpr = struct.unpack(tp,memchunk)
    return interpr



# Write in the shared instruction memory
def wshm(var,val):
    """ Writes the given value to the shared memory address associated
    with the given variable var."""
    tp,offset,size = find_variable(var, 'instr')
    if (len(tp) == 1):
        shm['instr'][(offset):(offset+size)] = struct.pack(tp,val)
    else :
        nb=int(tp[:-1])
        compl=[0 for i in range (nb-len(val))]
        val=np.concatenate((val,compl), axis=0)
        shm['instr'][(offset):(offset+size)] = struct.pack(tp,*val)
    shm['instr'].flush() #offset,size) # flush to ensure changes are written

# Write vectors in the shared instruction memory. Flush soon as all the values have been written.
#The purpose is to avoid flushing for each writing because it can take too much time.
def wshm_vector(var,val):
    """ Writes the given value to the shared memory address associated
    with the given variable var."""
    if (len (var)!=len(val)) :
        print "ERROR : The amount of variables is different of the amount of values. WSHM_VECTOR failed."
    else :
        for i in range (0, len (var)) :
            tp,offset,size = find_variable(var[i], 'instr')
            if (len(tp) == 1):
                shm['instr'][(offset):(offset+size)] = struct.pack(tp,val[i])
            else :
                nb=int(tp[:-1])
                compl=[0 for i in range (nb-len(val[i]))]
                val[i]=np.concatenate((val[i],compl), axis=0)
                shm['instr'][(offset):(offset+size)] = struct.pack(tp,*val[i])
        shm['instr'].flush() #offset,size) # flush to ensure changes are written
