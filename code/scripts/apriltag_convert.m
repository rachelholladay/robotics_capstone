% Upscales AprilTags to fixed size
% Author: Neil Jassal

filetype = 'png';
raw_dir = 'apriltags/raw/';
out_dir = 'apriltags/upscaled/';

upscale_size = [1000 1000];

directories = dir([raw_dir '/*.' filetype]);

for i = 1:length(directories)
   tag = rgb2gray(imread([raw_dir directories(i).name]));
   upscaled = imresize(tag, upscale_size, 'nearest');
   imwrite(upscaled, [out_dir directories(i).name]);
end


% tag = rgb2gray(imread('apriltags/raw/tag36_11_00007.png'));
% upscaled = imresize(tag, [1000, 1000], 'nearest');
% imshow(upscaled)
% imwrite(upscaled, 'apriltags/test.png');