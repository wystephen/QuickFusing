clear;
figure(1);grid on;hold on;
test=load('test.txt');
test_imu= load('text_imu.txt');
save_trace = test;
plot3(test_imu(:,1),test_imu(:,2),test_imu(:,3),'b*-');
plot3(save_trace(:,1),save_trace(:,2),save_trace(:,3),'*r-');
the_line = zeros(2,3);
the_line(1,:) = save_trace(1,:);
the_line(2,:) = save_trace(length(save_trace),:);
plot3(the_line(:,1),the_line(:,2),the_line(:,3),'b-');
% line([save_trace(1,1) save_trace(-1,1)],[save_trace(1,2) save_trace(-1,2)],[save_trace(1,3) save_trace(-1,3));
grid on;hold on;
% axis equal;



