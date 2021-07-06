%%
%marametter vehicle
m=1;
n=50;
M=5;
V=12;
R=0.0725;
W=0.24;
D=0.2;
H=0.5;
L=0.18;
D=0.001;
fm=0.002;
jm=0.00057;
jw=2*M*L*L/3;
Rr=3;
Ke=0.645;
g=9.81;
T=0.01;
%%
%marametter matrix
syms  x1 x2 x3 x4 x5 x6 u f1 f2 f3

assume(x4>=-pi/18);
assume(x4<=pi/18);

a1=(2*m + M)*R*R + 2*jw + 2*jm;
b1=M*L*R*cos(x4)-2*jm;
c1=M*L*R*x5*x5*sin(x4)+2*fm*x5-2*fm*x2-2*D*x2 +2*u;
a2=M*L*R*cos(x4)-2*jm;
b2=M*L*L+jw+2*jm;
c2=M*g*L*sin(x4)-(2*fm*x5-2*fm*x2+2*u);
AA=inv([a1 b1;a2 b2])\([c1;c2]) ;
x3=AA(1,:);
x6=AA(2,:);

f1=x2;
f2=x3;
f3=x5;
f4=x6;
%%
%caculateor matrix A ,B ,C

A= [
   subs(diff(f1,x1),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0])  subs(diff(f1,x2),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0])   subs(diff(f1,x4),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0])   subs(diff(f1,x5),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0]);
   subs(diff(f2,x1),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0])  subs(diff(f2,x2),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0])   subs(diff(f2,x4),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0])   subs(diff(f2,x5),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0]);
   subs(diff(f3,x1),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0])  subs(diff(f3,x2),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0])   subs(diff(f3,x4),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0])   subs(diff(f3,x5),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0]);
   subs(diff(f4,x1),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0])  subs(diff(f4,x2),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0])   subs(diff(f4,x4),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0])   subs(diff(f4,x5),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0])
   ];
B=[ subs(diff(f1,u),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0]);  
    subs(diff(f2,u),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0]); 
    subs(diff(f3,u),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0]);    
    subs(diff(f4,u),[x1,x2,x3,x4,x5,x6,u],[0,0,0,0,0,0,0])
   ];


%%
A_=zeros(4,4);
B_=zeros(4,1);
for i=1:1:4 
    for j=1:1:4
        A_(i,j)=A(i,j);
    end
    B_(i,1)=B(i,1);
end
%Ma tran cho bo LQR
Q =[1 0 0 0;0 1 0 0;0 0 10 0;0 0 0 1];
R=1;
P=icare(A_,B_,Q,R);
K=lqr(A_,B_,Q,R);
%A_(:,1)=1;
%[K,S,P] = lqr(A_,B_,Q,R);


%%
 %tim Jmin
 sim('LQR.mdl');
 x1=y.signals(1,1).values(:,1);
 x2= y.signals(1,1).values(:,2);
 x3=y.signals(1,2).values(:,1);
 x4= y.signals(1,2).values(:,2);
% 
 x=[x1(1);x2(1);x3(1);x4(1)];
 J_min=x'*P*x