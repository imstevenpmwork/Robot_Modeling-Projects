%% Computes the values of qa, qad and qadd needed to cross a Type 2 singularity
function [q] = singularity_crossing()
    
    %% Clear variables
    clc;
    close all;
    
    %% Geometric constants
    d= 0.4;
    l= 0.3606;
    
    %% Type 2 singularity position, velocity and acceleration
    % We calculated this point by imagining q1=0 and then move q2 down
    xs= 0;
    ys= -0.30005392848619727441672466738964;
    % xsdd and ysdd follows the criterion obtained
    ysdd= -0.1;
    xsdd= d*ysdd/(0.60010785697239454883344933477929);
    
    % xsdd and ysdd that don't follow the criterion
    %ysdd= -sqrt(2)/20;
    %xsdd= -pi()/30;
    
    % Using the same criterion for the velocities
    xsd=xsdd;
    ysd=ysdd;
    
    %% Variables for the trajectory
    % Intial position, velocity and acceleration
    xi= 0;
    yi= 0.3;
    xid= 0;
    xidd= 0;
    yid= 0;
    yidd= 0;
    
    % Type 1 singularity position, velocity and acceleration
    xm=0.1606;
    ym=0;
    ymdd=-0.1;
    xmdd=0;
    ymd=-0.2;
    xmd=0;
    
    % Final position, velocity and acceleration
    xf = -0.1;
    yf = -0.15;
    xfd = 0;
    xfdd = 0;
    yfd = 0;
    yfdd = 0;
   
    
    %% Time variables for the trajectory
    tf= 6;
    tm=2;
    ts=4;
    
    n = 1000;
    
    t1=linspace(0,tm,n);
    t2=linspace(tm,ts,n);
    t3=linspace(ts,tf,n);
    
    %% Generating the trajectory
    
    one_vector = ones(1,n);
    
    X1 = [xi;xid;xidd;xm;xmd;xmdd];
    X2= [xm;xmd;xmdd;xs;xsd;xsdd];
    X3= [xs;xsd;xsdd;xf;xfd;xfdd];
    
    Y1 = [yi;yid;yidd;ym;ymd;ymdd];
    Y2= [ym;ymd;ymdd;ys;ysd;ysdd];
    Y3=[ys;ysd;ysdd;yf;yfd;yfdd];
    
    A1 = [1, 0,  0,    0,      0,       0;
        0, 1,  0,    0,      0,       0;
        0, 0,  2,    0,      0,       0;
        1, tm, tm^2, tm^3,   tm^4,    tm^5;
        0, 1,  2*tm, 3*tm^2, 4*tm^3,  5*tm^4;
        0, 0,  2,    6*tm,   12*tm^2, 20*tm^3];
    
    A2= [1, tm, tm^2, tm^3,   tm^4,    tm^5;
        0, 1,  2*tm, 3*tm^2, 4*tm^3,  5*tm^4;
        0, 0,  2,    6*tm,   12*tm^2, 20*tm^3;
        1, ts, ts^2, ts^3,   ts^4,    ts^5;
        0, 1,  2*ts, 3*ts^2, 4*ts^3,  5*ts^4;
        0, 0,  2,    6*ts,   12*ts^2, 20*ts^3];
    
    A3= [1, ts, ts^2, ts^3,   ts^4,    ts^5;
        0, 1,  2*ts, 3*ts^2, 4*ts^3,  5*ts^4;
        0, 0,  2,    6*ts,   12*ts^2, 20*ts^3;
        1, tf, tf^2, tf^3,   tf^4,    tf^5;
        0, 1,  2*tf, 3*tf^2, 4*tf^3,  5*tf^4;
        0, 0,  2,    6*tf,   12*tf^2, 20*tf^3];
        
    % Solving for the coefficients of each trajectory
    ax1= A1\X1;
    ay1= A1\Y1;
    
    ax2= A2\X2;
    ay2= A2\Y2;
    
    ax3=A3\X3;
    ay3=A3\Y3;
    
    % Trajectory points
    xt1= ax1(1)*one_vector + ax1(2)*t1 + ax1(3)*t1.^2 + ax1(4)*t1.^3 + ax1(5)*t1.^4 + ax1(6)*t1.^5;
    yt1= ay1(1)*one_vector + ay1(2)*t1 + ay1(3)*t1.^2 + ay1(4)*t1.^3 + ay1(5)*t1.^4 + ay1(6)*t1.^5; 
    
    xt2= ax2(1)*one_vector + ax2(2)*t2 + ax2(3)*t2.^2 + ax2(4)*t2.^3 + ax2(5)*t2.^4 + ax2(6)*t2.^5;
    yt2= ay2(1)*one_vector + ay2(2)*t2 + ay2(3)*t2.^2 + ay2(4)*t2.^3 + ay2(5)*t2.^4 + ay2(6)*t2.^5;
    
    xt3= ax3(1)*one_vector + ax3(2)*t3 + ax3(3)*t3.^2 + ax3(4)*t3.^3 + ax3(5)*t3.^4 + ax3(6)*t3.^5;
    yt3= ay3(1)*one_vector + ay3(2)*t3 + ay3(3)*t3.^2 + ay3(4)*t3.^3 + ay3(5)*t3.^4 + ay3(6)*t3.^5; 

    
    % Plotting the three trajectories together
    plot(xt1,yt1);
    hold on;
    plot(xt2,yt2);
    plot(xt3,yt3);
    
    title('Type 1 & 2 Singularity Crossing');
    xlabel('x axis (meters)'); 
    ylabel('y axis (meters)'); 
    
    %% Solving the values of the joints that make the end effector follow the trajectory
    
    q11 = -1*sqrt(l^2 - (xt1 + d/2).^2) + yt1;
    q21= -1*sqrt(l^2 - (xt1 - d/2).^2) + yt1;
    
    % The sign of the joint q2 changes when we arrive the Type 1
    % singularity !
    q12= 1*sqrt(l^2 - (xt2 + d/2).^2) + yt2;
    q22 = -1*sqrt(l^2 - (xt2 - d/2).^2) + yt2;
    
    q13 = 1*sqrt(l^2 - (xt3 + d/2).^2) + yt3;
    q23= -1*sqrt(l^2 - (xt3 - d/2).^2) + yt3;
    
    
    %% Visualizating the results by feeding the values of the joints into our DGM
    % Not very efficient but working
    
    for i=1:length(q11)
        
        plot(xt1,yt1);
        
        title('Type 1 & 2 Singularity Crossing');
        xlabel('x axis (meters)'); 
        ylabel('y axis (meters)');
        axis([-0.2 0.2 -0.7 0.5]);
        
        hold on;
        plot(xt2,yt2);
        plot(xt3,yt3);
        plot(xm,ym,'r*');
        
        % before getting to the Type 2 singularity, gamma of the DGM is -1
        plot_lines(q11(i),q21(i),-1);
        hold off;
        
        %pause(0.05);
    end
    
    % Starting from the second element in order to exclude plotting the
    % final/starting point twice
    for i=2:length(q12)   
        
        plot(xt1,yt1);
        
        title('Type 1 & 2 Singularity Crossing');
        xlabel('x axis (meters)'); 
        ylabel('y axis (meters)');
        axis([-0.2 0.2 -0.7 0.5]);
        
        hold on;
        plot(xt2,yt2);
        plot(xt3,yt3);
        plot(xs,ys,'r*');
        
        % DGM gamma is still -1
        plot_lines(q12(i),q22(i),-1);
        hold off;
        
        %pause(0.05);
    end
    
    
    for i=2:length(q13)
        
        plot(xt1,yt1);
        
        title('Type 1 & 2 Singularity Crossing');
        xlabel('x axis (meters)'); 
        ylabel('y axis (meters)');
        axis([-0.2 0.2 -0.7 0.5]);
        
        hold on;
        plot(xt2,yt2);
        plot(xt3,yt3);
        
        % After the Type 2 singularity, gamma es set to +1
        plot_lines(q13(i),q23(i),+1);
        hold off;
     
        %pause(0.05);
    end
    
    %% Returning the values needed for the ADAMS simulation
    
    q1 = [q11, q12(2:end), q13(2:end)];
    q2 = [q21, q22(2:end), q23(2:end)];
    t = [t1, t2(2:end), t3(2:end)];

    Q1 = [t' , q1'];
    Q2 = [t' , q2'];

    save('q1.txt', 'Q1', '-ASCII','-append');
    save('q2.txt', 'Q2', '-ASCII','-append');
    
    q=[q1' q2'];

end

%% Function for the visualization
function [] = plot_lines(q1,q2,gamma)
    
    %% Geometric parameters
    d= 0.4;
    l= 0.3606;

    %% DGM
    OA = [d/2; q2];
    AM = [-d/2; (q1-q2)/2];
    MC = gamma * sqrt((4*l^2 - d^2 -(q1-q2)^2)/(d^2 + (q1-q2)^2)) * [0 -1; 1 0] * AM;
    X= OA + AM + MC;
    
    %% Ploting the arms and endeffector movement
    A1= [-d/2; q1];
    A2= [d/2; q2];
    
    hold on;
    plot([A1(1),X(1)],[A1(2),X(2)]);
    plot([A2(1),X(1)],[A2(2),X(2)]);
    plot(X(1),X(2),'g*');
    
end