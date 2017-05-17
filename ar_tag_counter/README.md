# ar_tag_counter

### Required packages
ar_track_alvar - http://wiki.ros.org/ar_track_alvar

### Published topics
**/tag_count** _(std_msgs/Int32)_

Publishes the last valid count of AR tags

### Subscribed topics
**/ar_pose_marker** _(ar_track_alvar/AlvarMarkers)_

This is a list of the poses of all the observed AR tags, with respect to the output frame

### Services provided
**/reset** _(std_srvs/Empty)_

Service used to manually reset the counter

### Parameter
**/ar_tag_number** _(int, default 9)_

Number of possible detectable AR tags

**/use_timer** _(bool, default true)_

Enbale the automatic reset of the counter based on a timer

**/reset_time** _(double, default 120.0)_

Number of seconds between each automatic reset
