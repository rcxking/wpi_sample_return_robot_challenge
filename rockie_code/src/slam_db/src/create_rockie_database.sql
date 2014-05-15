CREATE DATABASE rockie;

USE rockie;

CREATE TABLE image_frame (
frame_id INT NOT NULL AUTO_INCREMENT,
frame_pair_id INT,
stereo_is_left BIT(1),
capture_time DATETIME,
filepath VARCHAR(1000),
PRIMARY KEY (frame_id) );

CREATE TABLE image_frame_keypoint (
keypoint_id INT NOT NULL AUTO_INCREMENT,
frame_id INT,
keypoint_data VARCHAR(100),
PRIMARY KEY (keypoint_id)
);

#DROP DATABASE rockie;