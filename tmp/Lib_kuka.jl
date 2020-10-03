####
#
#  Librairie Kuka, philippe.fraisse@lirmm.fr
#####

using LinearAlgebra
mutable struct DH
    α::Array{Float64,1}
    d::Array{Float64,1}
    r::Array{Float64,1}
    m::Array{Float64,1}
    c::Array{Float64,2}
end

function CreateRobotKukaLwr()
 α=[0.0,pi/2,-pi/2,-pi/2,pi/2,pi/2,-pi/2];
 d=zeros(7);
 r=[0.3105,0.0,0.4,0.0,0.39,0.0,0.078]
 θ=zeros(7);
 m=[2.7,2.7,2.7,2.7,2.7,2.7,0.3];
 c=zeros(4,7)
 c[2,:]=[-8.70e-3, 8.7e-3, 8.7e-3, -8.7e-3,-8.2e-3, -7.6e-3, 0.0];
 c[3,:]=[-1.461e-2,1.461e-2,-1.461e-2, 1.461e-2,-3.48e-2, 1.363e-3, 0.0];
 c[4,:]=[1.0,1.0,1.0,1.0,1.0,1.0,1.0];
 robot_kuka=DH(α,d,r,m,c)
 return robot_kuka
end

function MGD(θ,robot)
    N=length(robot.α);
    T=zeros(4,4);
    Tp=Matrix(I,4,4);
     for i=1:N
        cθ=cos(θ[i]); sθ=sin(θ[i]); d=robot.d[i]; sα=sin(robot.α[i]); cα=cos(robot.α[i]); r=robot.r[i];
        T=Tp*[ cθ     -sθ    0.0    d;
              cα*sθ  cα*cθ   -sα   -r*sα;
              sα*sθ  sα*cθ    cα    r*cα;
               0.0    0.0     0.0    1.0  ];
        Tp=T;
    end
    return T
end

function Jacobian(θ,robot,p0)
    N=length(robot.α);
    T=zeros(4,4); l=zeros(3,N); z=zeros(3,N);
    Tp=Matrix(I,4,4);
    J=zeros(6,N);
    for i=1:N
        cθ=cos(θ[i]); sθ=sin(θ[i]); d=robot.d[i]; sα=sin(robot.α[i]); cα=cos(robot.α[i]); r=robot.r[i];
        T=  Tp*  [ cθ     -sθ    0.0    d;
              cα*sθ  cα*cθ   -sα   -r*sα;
              sα*sθ  sα*cθ    cα    r*cα;
               0.0    0.0     0.0    1.0 ];
       display(T)
       l[:,i]=p0[:]-T[1:3,4];
       z[:,i]=T[1:3,3];
       J[1:3,i]=cross(z[:,i],l[:,i]);
       J[4:6,i]=z[:,i];
       Tp=T;
    end
    return J
end


function CoM(θ,robot)
    """ test """
        N=length(robot.α);
        M=sum(robot.m[1:N]);
        T=zeros(4,4);
        CoM=zeros(4,1);CoMp=zeros(4,1);
        Tp=Matrix(I,4,4);
        for i=1:N
            cθ=cos(θ[i]); sθ=sin(θ[i]); d=robot.d[i]; sα=sin(robot.α[i]); cα=cos(robot.α[i]); r=robot.r[i];
            T=Tp*[ cθ     -sθ    0.0    d;
                  cα*sθ  cα*cθ   -sα   -r*sα;
                  sα*sθ  sα*cθ    cα    r*cα;
                  0.0    0.0     0.0    1.0  ];
            CoM[:]=(robot.m[i]/M).*T*robot.c[:,i]+CoMp;
            CoMp=CoM;
            Tp=T;
        end
        return CoM
end

function JacobianCoM(θ,robot,CoM0)
N=length(robot.α);
M=sum(robot.m[1:N]);
CoM=zeros(4,1);CoMp=zeros(4,1);
T=zeros(4,4); l=zeros(3,N); z=zeros(3,N);
Tp=Matrix(I,4,4);
CoMo=zeros(4,1);
Jcom=zeros(3,N);
for i=1:N
    cθ=cos(θ[i]); sθ=sin(θ[i]); d=robot.d[i]; sα=sin(robot.α[i]); cα=cos(robot.α[i]); r=robot.r[i];
    T= Tp*[ cθ     -sθ    0.0    d;
          cα*sθ  cα*cθ   -sα   -r*sα;
          sα*sθ  sα*cθ    cα    r*cα;
           0.0    0.0     0.0    1.0 ];
   CoMo[:]=(robot.m[i]/M).*T*robot.c[:,i];
   l[:,i]=CoM0[1:3]-CoMo[1:3];
   z[:,i]=T[1:3,3];
   Jcom[1:3,i]=cross(z[:,i],l[:,i]);
   Tp=T;
end
return Jcom
end

function readsensors(objectname)
    N=length(objectname);
    qread=zeros(N);
    qread =getjointposition(clientID,7,0,objectname_kuka);
    return qread
end

function InvKinControl(xd,robot)
    N=length(robot.α);  T=zeros(4,4);
    J=zeros(6,N); JP=zeros(3,N); qm=zeros(N);dot_θ=zeros(N);
    #--------------
    qm=readsensors(objectname_kuka)
    T=MGD(qm,robot);
    #--------------
    J=Jacobian(qm,robot,T[1:3,4]);
    JP=J[1:3,:];
    dot_θ=pinv(JP)*(xd-T[1:3,4]);
    return dot_θ
end

function InvKinControlCoM(CoMd,robot)
    N=length(robot.α);  CoMm=zeros(4,1);
    Jcom=zeros(3,N); JP=zeros(3,N); qm=zeros(N);dot_θ=zeros(N);
    #--------------
    qm=readsensors(objectname_kuka)
    CoMm=CoM(qm,robot);
    #--------------
    Jcom=JacobianCoM(qm,robot,CoMm);
    dot_CoM=pinv(Jcom)*(CoMd-CoMm[1:3]);
    return dot_CoM
end
