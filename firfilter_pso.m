clc;
clear;
close all;


%% Parameters of PSO
obj=@(x) objfun(x);
MaxIt = 250;           % Maximum Number of Iterations
order = 20;
Varsize = 30;
nPop = 25;              % Population Size (Swarm Size)  Size of the frequency list
w = 0.3;                % Intertia Coefficient
wdamp = 0.999;          % Damping Ratio of Inertia Coefficient
c1 = 2;                 % Personal Acceleration Coefficient
c2 = 2;                 % Social Acceleration Coefficient
ShowIterInfo = true;    % Flag for Showing Iteration Informatin
isfirstPrint = false;

MaxPosition = 2;
MinPosition = -2;
MaxVelocity = 2;
MinVelocity = -2;



%% design low pass filter

%desiredFilter = fir1(31,0.5,kaiser(32,8));

%[desiredFilter_h, desiredFilter_w] = freqz(desiredFilt);

%figure('name', 'PSO on FIR order 31');
%plot(desiredFilter_w/pi ,20*log10(abs(desiredFilter_h)));
%load chirp
%t = (0:length(y)-1)/Fs;
%desiredFilter= fir1(34,0.48,'high',chebwin(35,30));
%[desiredFilter_h, desiredFilter_w]=freqz(desiredFilter,1,250);

%plot(desiredFilter_w/pi ,(abs(desiredFilter_h)));

desiredFilter = fir1(34,0.48,chebwin(35,30));
[desiredFilter_h, desiredFilter_w]=freqz(desiredFilter,1,250);
plot(desiredFilter_w/pi ,abs(desiredFilter_h));


%% Initialization

% The Particle Template
particle.Position = [];
particle.Velocity = [];
particle.Best.Position = [];
particle.Cost = [];
particle.Best.Cost = [];
GlobalBest.Cost=inf;


% Create Population Array
%particle = repmat(particle, nPop, 1);

% Initialize Global Best
GlobalBest.Position = unifrnd(MinPosition, MaxPosition,1,Varsize);

% Initialize Population Members
for i=1:nPop

    % Generate Random Solution
    particle(i).Position = unifrnd(MinPosition, MaxPosition, 1, Varsize);
    particle(i).Velocity = unifrnd(MinVelocity, MaxVelocity, 1,Varsize);
    
    % Evaluation
    particle(i).Cost = obj(particle(i).Position);
   
    particle(i).Best.Position = particle(i).Position;
    particle(i).Best.Cost =  particle(i).Cost;
   
    if findError(particle(i).Position, desiredFilter_h) < findError(GlobalBest.Position, desiredFilter_h)
        GlobalBest.Position  = particle(i).Position;
    end

end
BestCost=zeros(MaxIt,1);

%% Main Loop of PSO
for it=1:MaxIt

    for i=1:nPop
       
        % Update Velocity
        for j = 1:Varsize
            particle(i).Velocity(j) = w*particle(i).Velocity(j) ...
                + c1*rand*(particle(i).Best.Position(j) - particle(i).Position(j)) ...
                + c2*rand*(GlobalBest.Position(j) - particle(i).Position(j));
        end
        
        % Update Position
        particle(i).Position = particle(i).Position + particle(i).Velocity;
        
        % Update Local Best
        if findError(particle(i).Position, desiredFilter_h) < findError(particle(i).Best.Position, desiredFilter_h)
            particle(i).Best.Position = particle(i).Position;
            
        % Evaluation
       particle(i).Cost = obj(particle(i).Best.Position);
        
        % Update Personal cost
        if particle(i).Cost < particle(i).Best.Cost
            particle(i).Best.Cost=particle(i).Cost;
        end
        %  update global cost
          if GlobalBest.Cost > particle(i).Best.Cost
              GlobalBest.Cost = particle(i).Best.Cost;
          end
            % Update Global Best
            if findError(particle(i).Best.Position, desiredFilter_h) < findError(GlobalBest.Position, desiredFilter_h)
                GlobalBest.Position = particle(i).Best.Position;
            end 
        end
    end
    BestCost(it)=GlobalBest.Cost;
 
    
    % Display Iteration Information
    if ShowIterInfo
        hold all

        [temp_y,temp_x] = freqz(GlobalBest.Position,1 ,250);
        
        % Check if first entry
        if(isfirstPrint)
            delete(mt);
        end
        
         mt=plot(temp_x/pi,(abs(temp_y)));
        
        title({'Interation #:' num2str(it) ('pso with lowpass')});
       
        xlabel('w ( x pi)');
        ylabel('|H(jw)| (abs)');
        
        hold off
        drawnow
        isfirstPrint = true;
    end
     

    % Damping Inertia Coefficient
    w = w * wdamp;
end
 disp('answer:')
    for i=1:nPop
       disp(particle(i).Position)
    end
    
plot(BestCost);
xlabel('iteration');
ylabel('error fitness');
disp('Done!');