clear all
clc

A= [1 -1.5 0.56];
B=[0.9 -0.6];

% Propogate A matrix with z 
A_tilda=conv(A,[1 -1])

% Dividing 1 over A_tilda yields:
num=[1 0 0 0];
E_coeffs= ldiv(num,A_tilda,10)


sr=zeros(10,10);
for i=1:10
    for j=1:i
    E1_10(i,j)=[E_coeffs(j)]
    end
end

Bolunen=[1 0 0 0 zeros(1,10)]
Bolen= A_tilda
Bolum=E_coeffs
F_coeffs=zeros(10,3)
for i=1:10 
    Kalan= Bolunen(i:i+3)-(Bolum(i)*Bolen)
    Bolunen=[zeros(1,i-1) Kalan zeros(1,10-i)]
    F_coeffs(i,:)=[Kalan(2:4)]
end

Ge=zeros(10,11)
for k=1:10
    Ge(k,:)=[conv(E1_10(k,1:k),B) zeros(1,10-k)]
end

G=zeros(10,10)
for m=1:10 
   G(m,:)=[Ge(m,m) sum(Ge(m,1:m-1)) zeros(1,8)]
   ut_1C(m,:)=[Ge(m,m+1)]    
end

u(1)=0; u(2)=0;
y(1)=0; y(2)=0; y(3)=0;
W(1)=0;
timer=1;
lambda=0.8;
alpha=0.1;
r=ones(10,1);

for t=3:100
while timer<10
        W(timer+1,1)=alpha*W(timer,1)+(1-alpha)*r(length(W),:);
        timer=timer+1;
end

    f= ut_1C*u(t-1)
    for n=1:10
       f(n)=f(n,:)+ F_coeffs(n,1)*y(t) + F_coeffs(n,2)*y(t-1) + F_coeffs(n,3)*y(t-2)
    end

    U=inv(transpose(G(:,1:2))*G(:,1:2)+lambda*eye(2))*transpose(G(:,1:2))*(W-f)

    Y=G(:,1:2)*U+f

y(t+1)=Y(1);
u(t)=U(1);
Wfinal(t)=W(1);

%update

 W(1,:) = [];
 timer=1;
end

y(11)=[];
interval=1:length(y);
subplot(211)
plot(interval,y);
title('Output(y)')
subplot(212)
plot(interval,u);
title('Input(u)')
subplot(221)
plot(interval,Wfinal);
subplot(222)
plot(interval,y);
hold on
plot(interval,Wfinal);
hold off
