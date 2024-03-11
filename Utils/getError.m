function [Terror,Rerror] = getError(H_gt,H_est)

T_gt = H_gt(1:3,4);
T_est = H_est(1:3,4);
Terror = sqrt(sum((T_gt-T_est).^2));

R_gt = H_gt(1:3,1:3);
R_est = H_est(1:3,1:3);

traza = trace(R_gt'*R_est);
corrected_value = min(max((traza-1)/2,-1),1);
Rerror = abs(acos(corrected_value));

%R_gt_euler = rotm2eul(H_gt(1:3,1:3));
%R_est_euler = rotm2eul(H_est(1:3,1:3));

%Rerror = sqrt(sum((R_gt_euler-R_est_euler).^2));
end