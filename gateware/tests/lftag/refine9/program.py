"""Schedule the block-Schur nine-point fit on a two-read-port MAC engine.

Each instruction writes RAM[pc]; operands are previous results or feature
vectors. Vector/dot instructions reuse the MAC without instruction fetches.
The 20-bit instruction is {opcode[4:0], address_a[7:0], address_b[6:0]}.
Vector operands B have an implicit high bit; scalar operands B are below 128.
"""
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
Q=17
GUARD=6
ONE=1<<Q
X,Y,U,V,UX,UY,VX,VY,ONES=(128+9*i for i in range(9))
SX,SY,CX,CY,S33,UNIT,ZERO=range(209,216)
# Bits 3:2 encode the multiplier scale, bit 0 retains the accumulator,
# and bits 1:0 == 3 subtract. Bit 4 distinguishes the other instructions.
# Calibration and Schur reciprocal products have the same binary scale.
OPS={'MUL':0,'MAC':1,'MSUB':3,'DOT9':4,'DOT9_ADD':5,'DOT3':6,
     'DOT3_SUB':7,'MUL8':8,'MSUB8':11,'CAL':12,'MUL_R':12,
     'RCP':16,'COPY':17,'ADD':18,'SUB':19,'CONFIG':20,'OUTPUT':21,
     'VECTOR':22,'HALT':23}
program=[]
listing=[]
def emit(op,a=0,b=0):
    if op in ('DOT9','DOT9_ADD','VECTOR'):
        assert 128<=b<256
        b-=128
    assert 0<=a<256 and 0<=b<128,(op,a,b,len(program))
    address=len(program)
    program.append((OPS[op]<<15)|(a<<7)|b)
    listing.append((op,a,b))
    return address
def copy(a):return emit('COPY',a)
def mul(a,b):return emit('MUL',a,b)
def dot(a,b):return emit('DOT9',a,b)
def difference(a,b,c,d):
    mul(a,b)
    return emit('MSUB',c,d)
def vector(a):
    first=len(program)
    for v in a:copy(v)
    return first

# Camera scalars have a disjoint lifetime from the normal equations. They
# remain at low addresses so both final homography operands are encodable.
IFX=emit('CONFIG',0);IFY=emit('CONFIG',1)
PCX=emit('CONFIG',2);PCY=emit('CONFIG',3)
AX=emit('CAL',SX,IFX);AY=emit('CAL',SY,IFY)
temp=emit('SUB',CX,PCX);BX=emit('CAL',temp,IFX)
temp=emit('SUB',CY,PCY);BY=emit('CAL',temp,IFY)
for a,b in ((U,X),(U,Y),(V,X),(V,Y)):emit('VECTOR',a,b)

# S=W'W/8, C=W'diag(u or v)[X,Y]/8, b=W'u/8.
s00=dot(X,X);s01=dot(X,Y);s02=dot(X,ONES)
s11=dot(Y,Y);s12=dot(Y,ONES)
bu=[dot(U,a) for a in (X,Y,ONES)]
bv=[dot(V,a) for a in (X,Y,ONES)]
cu0=[dot(UX,X),dot(UX,Y),copy(bu[0])]
cu1=[copy(cu0[1]),dot(UY,Y),copy(bu[1])]
cv0=[dot(VX,X),dot(VX,Y),copy(bv[0])]
cv1=[copy(cv0[1]),dot(VY,Y),copy(bv[1])]
assert all(v==list(range(v[0],v[0]+3)) for v in (bu,bv,cu0,cu1,cv0,cv1))
d=[]
for a,b,c,e in ((UX,UX,VX,VX),(UX,UY,VX,VY),(UY,UY,VY,VY),
                (UX,U,VX,V),(UY,U,VY,V)):
    dot(a,b);d.append(emit('DOT9_ADD',c,e))

# Invert only the shared 3x3 S. Its well-conditioned world coordinates do
# not depend on perspective or tag size.
k00=difference(S33,s11,s12,s12)
k01=difference(s02,s12,S33,s01)
k02=difference(s01,s12,s02,s11)
k11=difference(S33,s00,s02,s02)
k12=difference(s01,s02,s00,s12)
k22=difference(s00,s11,s01,s01)
kr=vector((k00,k01,k02))
# s00,s01,s02 were emitted consecutively.
det=emit('DOT3',s00,kr);inverse=emit('RCP',det,0)
ki=[mul(k,inverse) for k in (k00,k01,k02,k11,k12,k22)]
rows=[vector([ki[i] for i in row]) for row in ((0,1,2),(1,3,4),(2,4,5))]
solutions=[]
for rhs in (cu0,cu1,bu,cv0,cv1,bv):
    solutions.append([emit('DOT3',row,rhs[0]) for row in rows])
au0,au1,zu,av0,av1,zv=solutions

# 2x2 Schur system, retaining the full nine-point algebraic least-squares
# objective. C has positive sign here; the original off-diagonal block is -C.
t=[]
for diagonal,ca,za,cb,zb in ((d[0],cu0,au0,cv0,av0),
                            (d[1],cu0,au1,cv0,av1),
                            (d[2],cu1,au1,cv1,av1),
                            (d[3],cu0,zu,cv0,zv),
                            (d[4],cu1,zu,cv1,zv)):
    copy(diagonal);emit('DOT3_SUB',ca[0],za[0]);t.append(emit('DOT3_SUB',cb[0],zb[0]))
# The last two values are r-C' S^-1 b, the negative of the desired RHS.
emit('MUL8',t[0],t[2]);det=emit('MSUB8',t[1],t[1])
inverse2=emit('RCP',det,1)
gh=[]
for a,b,c,e in ((t[4],t[1],t[3],t[2]),(t[3],t[1],t[4],t[0])):
    numerator=difference(a,b,c,e);gh.append(emit('MUL_R',numerator,inverse2))
assert max(gh)<128,gh

h=[]
for first,second,offset in ((au0,au1,zu),(av0,av1,zv)):
    for i in range(3):
        mul(first[i],gh[0]);emit('MAC',second[i],gh[1]);h.append(emit('ADD',offset[i]))

# K^-1 and native-plane scaling. Alpha/beta are already Q17.
for axis in range(2):
    for i in range(3):
        mul(h[axis*3+i],(AX,AY)[axis])
        value=emit('MAC',(gh+[UNIT])[i],(BX,BY)[axis])
        emit('OUTPUT',value,axis*3+i)
for i,value in enumerate(gh+[UNIT]):emit('OUTPUT',value,6+i)
emit('HALT')
# Persistent constants must survive every job, including back-to-back fits.
assert len(program)<=ONES,len(program)

def checked(v):
    v=int(v)
    if not -(1<<19)<=v<(1<<19):raise ArithmeticError(('signed20',v))
    return v

def rounded(value,shift):return (value+(1<<(shift-1)))>>shift

def reference(tag_id,points,invf=(1533917,1533917),principal=(640,400),scale=0,trace=False):
    from model import world_points
    points=np.asarray(points,dtype=np.int64);lo=points.min(0);hi=points.max(0);center=(lo+hi)//2
    shifts=[int((h-l)//2+1).bit_length() for l,h in zip(lo,hi)]
    world=np.rint(world_points(tag_id)*ONE).astype(np.int64)
    memory=[0]*512
    for axis in range(2):
        for n in range(9):
            memory[(X,Y)[axis]+n]=int(world[n,axis])
            memory[(U,V)[axis]+n]=int(points[n,axis]-center[axis])<<(Q-shifts[axis])
        memory[(SX,SY)[axis]]=1<<(shifts[axis]+2*scale)
        memory[(CX,CY)[axis]]=int(center[axis])*(1<<(2*scale))+(64 if axis==1 and scale==2 else 0)
    for n in range(9):memory[ONES+n]=ONE
    memory[S33]=9<<(Q-3);memory[UNIT]=ONE
    acc=0;cycles=0
    for pc,(op,a,b) in enumerate(listing):
        if op=='HALT':break
        if op=='CONFIG':
            acc=checked(invf[a]>>5 if a<2 else principal[a-2]*16);cycles+=3
        elif op=='VECTOR':
            for n in range(9):memory[(UX,UY,VX,VY)[pc-10]+n]=checked(rounded(memory[a+n]*memory[128+b+n],Q))
            cycles+=19;continue
        elif op.startswith('DOT'):
            length=9 if op.startswith('DOT9') else 3
            if length==9:b+=128
            if op in ('DOT9','DOT3'):acc=0
            shift=Q+(3 if length==9 else 0)
            full=(acc<<GUARD)+(1<<(GUARD-1))
            for n in range(length):
                product=(memory[a+n]*memory[b+n])>>(shift-GUARD)
                full=full-product if op=='DOT3_SUB' else full+product
            acc=checked(full>>GUARD)
            cycles+=length+2
        elif op=='RCP':
            if memory[a]<=0:raise ArithmeticError('nonpositive determinant')
            acc=checked((1<<(Q+(15 if b else 17)))//memory[a]);cycles+=23
        elif op=='COPY':acc=memory[a];cycles+=2
        elif op=='ADD':acc=checked(acc+memory[a]);cycles+=2
        elif op=='SUB':acc=checked(memory[a]-memory[b]);cycles+=2
        elif op=='OUTPUT':
            value=rounded(memory[a],Q-16)
            if not -(1<<17)<=value<(1<<17):raise ArithmeticError('H output range')
            memory[256+2*b]=value;cycles+=2;continue
        else:
            shift=29-Q if op=='CAL' else 12 if op=='MUL_R' else Q-3 if op in ('MUL8','MSUB8') else Q
            product=(memory[a]*memory[b])>>(shift-GUARD)
            initial=(acc<<GUARD) if op in ('MAC','MSUB','MSUB8') else 0
            full=initial+(1<<(GUARD-1))+(-product if op in ('MSUB','MSUB8') else product)
            acc=checked(full>>GUARD)
            cycles+=3
        memory[pc]=acc
    if trace:return memory,cycles
    return [memory[256+2*i] for i in range(9)]

def write():
    p=ROOT/'rtl/lftag/lftag_fit9_program.svh'
    p.write_text('// Generated by tests/lftag/refine9/program.py.\n'+''.join(
        f"job_memory[{256+i}] = 20'h{value:05x}; // {listing[i]}\n" for i,value in enumerate(program)))
    constants={**{ONES+i:ONE for i in range(9)},S33:9<<(Q-3),UNIT:ONE,ZERO:0}
    (ROOT/'rtl/lftag/lftag_fit9_constants.svh').write_text(
        '// Persistent constants shared by fit and pose scratch RAM.\n'+''.join(
            f"bank_a[{a}] = 18'h{v&0x3ffff:05x}; bank_b[{a}] = 18'h{(v>>18)&3:05x}; mirror[{a}] = 20'h{v&0xfffff:05x};\n" for a,v in constants.items()))
    print('control words',len(program),'g/h addresses',gh)

if __name__=='__main__':
    from model import fixtures,world_points
    import test_accuracy
    worst=0;failures=0;cycles=[]
    for tag_id,points,truth in fixtures(500):
        try:memory,c=reference(tag_id,points,trace=True)
        except ArithmeticError as e:failures+=1;print('FAIL',tag_id,e);continue
        cycles.append(c)
        w=world_points(tag_id);lo=points.min(0);hi=points.max(0);center=(lo+hi)//2
        scale=np.array([2**int((h-l)//2+1).bit_length() for l,h in zip(lo,hi)])
        t=np.array([[scale[0]/16,0,center[0]/16],[0,scale[1]/16,center[1]/16],[0,0,1.]])
        expected=t@test_accuracy.fit_float(w,(points-center)/scale)
        focal=(1<<30)/1533917;k=np.array([[focal,0,640],[0,focal,400],[0,0,1.]])
        actual=k@(np.array([memory[256+2*i] for i in range(9)]).reshape(3,3)/65536)
        worst=max(worst,float(np.max(np.abs(test_accuracy.project(actual,w)-test_accuracy.project(expected,w)))))
    print('program words',len(program),'g/h',gh,'scheduled cycles',max(cycles),'max coordinate error',worst,'failures',failures)
