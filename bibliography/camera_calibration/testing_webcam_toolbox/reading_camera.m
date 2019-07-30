cameraObj2 = videoinput('winvideo', 2);
triggerconfig(cameraObj1, 'manual');
start(cameraObj1);
triggerconfig(cameraObj2, 'manual');
start(cameraObj2);

f1 = figure;
f2 = figure;
for i=1:5
    tic;
    img1 = getsnapshot(cameraObj1);
    img2 = getsnapshot(cameraObj2);
    [centers, radii, metric] = imfindcircles(img2,[15 60]);
     set(0, 'CurrentFigure', f1);
    viscircles(centers, radii,'EdgeColor','b');
%     set(0, 'CurrentFigure', f1);
%     imshow(img1);
    set(0, 'CurrentFigure', f2);
    imshow(img2);
    toc;
    pause(2);
end
delete(cameraObj2);
delete(cameraObj1);