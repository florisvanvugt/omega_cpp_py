import os
import mmap
import struct

def init_specifications():
    """Read the shared_memory_specification.txt file and parse it """
    f = open("shared_memory_specification.txt", "r")

    #Creation d'un dictionnaire avec tous les elements necessaires par la suite
    contenu = f.read()
    lignes = contenu.split("\n")
    address = 0 # this variable keeps track of the address in shared memory where we currenlty are
    global specifications
    specifications = {}
    for i in lignes:
        if (i != ''):
            variable = i.split(" ")
            if len(variable)==2:
                varname,vartype = variable
                sz = struct.calcsize(vartype)
                specifications[varname] = (vartype,address,sz)
                address += sz   # keep our running count of the memory offset
            else:
                print("Not sure what to do with line '%s'"%i)
    f.close()



def get_specifications():
    return specifications


def compute_address(offset, typevar): # DEPRECATED
    if (typevar == 'i'):
        return (offset + 4)
    elif (typevar == 'd'):
        return (offset + 8)
    elif (len(typevar)>1):
        type_tab, size_tab = typevar.strip(']').split('[')
        if (type_tab == 'i'):
            return (offset + 4*int(size_tab))
        elif (type_tab == 'd'):
            return (offset + 8*int(size_tab))
        #print("Type of the the variable is not good.")
        return 0




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
    #return((specifications[var])[1])


def rshm(var):
    """ Reads the shared memory value associated with the given variable,
    for example rshm('x') """
    tp,offset,size = find_variable(var)
    #typevar = (specifications[var])[0]
    #sizevar = compute_address(0, typevar)

    # Extract the memory chunk associated with this variable (still needs to be decoded)
    memchunk = shm[offset:(offset+size)]
    
    # Interpret the raw binary as the correct data type
    if (len(tp) == 1):
        interpr, = struct.unpack(tp,memchunk)

    else:
        interpr = struct.unpack(tp,memchunk)
    return interpr

    #if (len(typevar)>1):
    #        type_tab, size_tab = typevar.strip(']').split('[')
    #        return ((struct.unpack(str(size_tab)+type_tab, shm[offset:(offset+sizevar)])))
    #    else:
    #        return ((struct.unpack(typevar[0], shm[offset:(offset+sizevar)]))[0])



def wshm(var,val):
    """ Writes the given value to the shared memory address associated
    with the given variable var."""
    tp,offset,size = find_variable(var)
    #typevar = (specifications[var])[0]
    #sizevar = compute_address(0, typevar)
    shm[(offset):(offset+size)] = struct.pack(tp,val)
