clear 
model=createpde(1);
% Importing the stl file
importGeometry(model,"cuboid.stl");

% Generating volumetric mesh
mesh_node_sizing = 1;
mesh = generateMesh(model,hmin=30);
x = mesh.Nodes(1,:);
y = mesh.Nodes(2,:);
z = mesh.Nodes(3,:);

P = mesh.Nodes';
% Calculating the com
x_com = mean(x);
y_com = mean(y);
z_com = mean(z);
% plot3(x_com,y_com,z_com,'x');

% Translating all the points such that com of the part is the origin of the
% axis
Tx = [1 0 0 -x_com;0 1 0 -y_com;0 0 1 -z_com;0 0 0 1];
P(:,4)=ones(size(P(:,1)));
req_P = Tx*P';
axis equal
hold on
% Generating the convex hull to remove the inner points
% Another reason is that suppose the object is concave, right now under the
% assumption that holding the object in the concave region is tough and
% requires more force, especially in human tactile feedback
% Right now for the time being using convhull if there is a better
% algorithm , we can see that later

x = req_P(1,:);
y = req_P(2,:);
z = req_P(3,:);
req_P(4,:) = [];

% Since after this translation the com is the origin changing it 0,0,0
x_com = 0;
y_com = 0;
z_com = 0;

plot3(x_com,y_com,z_com,'x');

[k1,av1] = convhull(x,y,z);
TR = triangulation(k1,req_P');
I = incenter(TR);
F = faceNormal(TR);

trisurf(k1,x,y,z,'FaceColor','cyan')

%This plots all the normals at the incentres of the triangles
quiver3(I(:,1),I(:,2),I(:,3), ...
     F(:,1),F(:,2),F(:,3),0.5,'color','r');


% Defining the basis and the friction cone

max_external_force = 10;
force_vectors_applied = 10*F;
friction_surface = 0.5;

% This model is for point contact with friction, not yet completed,need to
% research more on this
% First issue , should i take max friciton value, is it valid for force 
% optimization?
% Second issue is the 6x3 vector which makes it more complicated
% f3 is the applied force on the object
% f3 = 10;
% % let f1 = f2 , assuming the both fricitons are same in the local x and y
% % directions
% f1 = friction_surface*f3/sqrt(2);
% f2 = f1;
% Bci_fi = [f1 0 0;0 f2 0;0 0 f3; 0 0 0;0 0 0; 0 0 0];


%Taking the frictionless point contact model, for better understanding of
%our logic
Bci_fi = [0;0;1;0;0;0];

% Looping through the rows of the normal vectors
G = [];
for i=1:size(F,1)
    normal = [F(i,1) F(i,2) F(i,3)];
    p = [I(i,1) I(i,2) I(i,3)];
    % More efficient way, even works for small angles
    x_axis_angle = atan2(norm(cross(normal,[1 0 0])),dot(normal,[1 0 0]));
    y_axis_angle = atan2(norm(cross(normal,[0 1 0])),dot(normal,[0 1 0]));
    z_axis_angle = atan2(norm(cross(normal,[0 0 1])),dot(normal,[0 0 1]));
    
    % These transformations are for premultiplying with the changed
    % coordinates
    R_alpha = [cos(z_axis_angle) -sin(z_axis_angle) 0;sin(z_axis_angle) cos(z_axis_angle) 0;0 0 1];
    R_beta = [cos(y_axis_angle) 0 sin(y_axis_angle);0 1 0;-sin(y_axis_angle) 0 cos(y_axis_angle)];
    R_gamma = [1 0 0;0 cos(x_axis_angle) -sin(x_axis_angle);0 sin(x_axis_angle) cos(x_axis_angle)];
    R = R_alpha*R_beta*R_gamma;
    
    cross_G = [0 -p(3) p(2);p(3) 0 -p(1);-p(2) p(1) 0]*R;
    % The grasp map
    G_i= [R zeros(3,3);cross_G R];
    % Fo_i will be 6x3 vector because there are 2 frictional forces and 1
    % applied force and not 6x1
    Fo_i = G_i*Bci_fi;
    G = [G Fo_i];
end
% n = 1:size(G,2);
% C = nchoosek(n,3);
% %Taking frictionless point contacts
% wrench_ext = [0;0;-10;0;0;0];
% c = 0;
% for i=1:size(C,1)
%     row_i = C(i,:);
%     vector_1 = G(:,row_i(1));
%     vector_2 = G(:,row_i(2));
%     vector_3 = G(:,row_i(3));
%     H = [vector_1 vector_2 vector_3];
%     rank_H = rank(H);
%     M = [vector_1 vector_2 vector_3 wrench_ext];
%     rank_M = rank(M);
%     if rank_M>3
%         continue
%     end
% 
%     if rank_M==3
%         X = linsolve(H,-wrench_ext);
%         if size(X,1)==4
%         if X(4) ~=0
%             c =c+1;
%         end
%         end
%     end
% end
% disp(c)


