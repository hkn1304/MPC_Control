function p=dmc(p)
% DMC   Dynamic Matrix Control
% P=DMC(P) determines the dynamic matrix control (input change) based on
% the plant model (step response) and current measurement stored in the
% structure P. 
% Input:
%   P.sr - unit step response data
%   P.u  - current input, initially 0
%   P.v  - past input, initially empty
%   P.G  - dynamic matrix, set by the initial call
%   P.F  - matrix to calculate free response, set by the initial call
%   P.k  - DMC gain, set by the initial call
%   P.r  - reference (set point)
%   P.a  - reference smooth factor
%   P.p  - prediction horizon
%   P.m  - moving horizon
%   P.y  - current mrasurement
%   P.la - performance criterion weight, i.e. J = ||r-y|| + p.la*||du||
%          where du is the input change
% Output:
%   P.u  - new input for next step
%   P.f  - updated free response
%   P.G  - dynamic matrix, if it is the first step.
%   P.k  - DMC gain, if it is the first step
%

% length of step response
N=numel(p.sr);
P=p.p;

% initial setup
if isempty(p.v)
    % number of past inputs to keep
    n=N-P;
    % storage for past input
    p.v=zeros(n,1);
    % matrix to calculate free response from past input
    x=p.sr(1:n);
    p.F=hankel(p.sr(2:P+1),p.sr(P+1:N))-repmat(x(:)',P,1);
    % dynamic matrix
    p.G=toeplitz(p.sr(1:P),p.sr(1)*eye(1,p.m));
    % calculate DMC gain
    R=chol(p.G'*p.G+p.la*eye(p.m));
    K=R\(R'\p.G');
    % only the first input will be used
    p.k=K(1,:);
    p.u=0;
end
% free response
f=p.F*p.v+p.y;
% smooth reference
nr=numel(p.r);
if nr>=P
    ref=p.r(1:P);
else
    ref=[p.r(:);p.r(end)+zeros(P-nr,1)];
end
w=filter([0 (1-p.a)],[1 -p.a],ref,p.y);
% DMC input change
u=p.k*(w-f);
% past input change for next step
p.v=[u;p.v(1:end-1)];
% next input
p.u=p.u+u(1);
