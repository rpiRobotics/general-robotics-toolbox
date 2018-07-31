function [x,v,a,ta,tb,tf]=trapgen(xo,xf,vo,vf,vmax,amax,dmax,t)
  
% Generate trapezoidal velocity motion profile.

% inputs:
% xo: initial position
% xf: final position
% vo: initial velocity
% vf: final velocity
% vmax: velocity limit
% amax: upper acceleration limit (magnitude)
% dmax: lower acceleration limit (magnitude)
% t: sample time

% outputs:
% x: position at sample time t
% v: velocity at sample time t
% a: acceleration at sample time t
% ta: first switching time
% tb: second switching time
% tf: final time


  % vo and vf must be less than vmax
  if (abs(vo)>=abs(vmax))|(abs(vf)>=abs(vmax))
    error('vo or vf too large');
  end  

  vmax=sign(xf-xo)*vmax;

  if xf>xo
    am1=abs(amax);
    am2=-abs(dmax);
  else
    am1=-abs(dmax);
    am2=abs(amax);
  end

  ta=abs((vmax-vo)/am1);
  tf_tb=(vf-vo-am1*ta)/am2;
  tf=(xf-xo+.5*am1*ta^2-.5*am2*tf_tb^2)/(am1*ta+vo);
  tb=tf-tf_tb;

  if ((tf<2*ta)|(tb<ta))
    tapoly=[1 2*vo/am1 ((vf^2-vo^2)/2/am2+xo-xf)*2/(1-am1/am2)/am1];
    ta = -vo/am1 + sqrt((vo/am1)^2- ... 
	(2/(1-am1/am2)/am1)*((vf-vo)^2/2/am2+xo-xf));
    tf=(vf-vo-am1*ta)/am2+ta;
    tb=ta;
  end

    if t<ta
      a=am1;
      v=am1*t+vo;
      x=.5*am1*t^2+vo*t+xo;
    elseif t<tb
      a=0;
      v=am1*ta+vo;
      x=am1*(t*ta-.5*ta^2)+vo*t+xo;
    elseif t<=tf
      a=am2;
      v=am2*(t-tb)+am1*ta+vo;
      x=am1*(-.5*ta^2+t*ta)+am2*(.5*t^2+.5*tb^2-tb*t)+vo*t+xo;
    else
      a=0;
      v=0;
      x=xf;
    end

