import os
import mmap
import struct
import yaml


specification_file = 'shared_memory_specification.yaml'


# This variable tells us where the variables are to be found
specifications = {}

# The order of the variables
variable_order = []


def init_specifications(basedir):
    """
    Read the shared_memory_specification.txt file and parse it
    """
    memspecf = "%s/%s"%(basedir,specification_file)
    print("Reading memory specification from %s"%memspecf)

    with open(memspecf,'r') as f:
        memsp = yaml.load(f)

    address = 0 # this variable keeps track of the address in shared memory where we currenlty are
    global specifications # start building a memory specification variable
    specifications = {}
    global variable_order # keep track of the order of the variables (should be alphabetical but we want to make sure this is right)
    variable_order = []
    
    for varname,vartype in memsp.iteritems():
        sz = struct.calcsize(vartype)
        specifications[varname] = (vartype,address,sz)
        address += sz   # keep our running count of the memory offset
        variable_order.append(varname)



def get_specifications():
    return specifications



#Open the shared memory
def init_shared_memory():
    """ Initialise shared memory, connect to shared memory of a C++ process """
    global shm
    fp = os.open("/tmp/sharedmemory.bin",os.O_RDWR)
    shm = mmap.mmap(fp, 0, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)


def find_variable(var):
    """ Find the memory and data type associated with the variable var """
    if var not in specifications:
        print("ERROR: unknown variable %s"%var)
        exit()
    return specifications[var]



def rshm(var):
    """ Reads the shared memory value associated with the given variable,
    for example rshm('x') """

    if isinstance(var, list):
        return [rshm(v) for v in var]
    
    tp,offset,size = find_variable(var)

    # Extract the memory chunk associated with this variable (still needs to be decoded)
    memchunk = shm[offset:(offset+size)]
    
    # Interpret the raw binary as the correct data type
    if (len(tp) == 1):
        interpr, = struct.unpack(tp,memchunk)
    else:
        interpr = struct.unpack(tp,memchunk)
    return interpr




def wshm(var,val):
    """ Writes the given value to the shared memory address associated
    with the given variable var."""
    tp,offset,size = find_variable(var)
    shm[(offset):(offset+size)] = struct.pack(tp,val)
    shm.flush() #offset,size) # flush to ensure changes are written
