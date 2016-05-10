function kin = combine_kinematics(kinA, kinB)
    % COMBINE_KINEMATICS
    %
    % kin = combine_kinematics(kinA, kinB)
    %
    % attaches the kinematics from kinB at the end of the kinematic chain
    % of kinA and returns a single structure containing the full chain
    %
    % see also FWDKIN
    
    kin.H = [kinA.H kinB.H];
    kin.P = [kinA.P(:,1:end-1) kinA.P(:,end)+kinB.P(:,1) kinB.P(:,2:end)];
    kin.joint_type = [kinA.joint_type kinB.joint_type];
end