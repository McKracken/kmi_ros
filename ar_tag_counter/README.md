---Required packages---
ar_track_alvar - http://wiki.ros.org/ar_track_alvar

---Published topics---
/tag_count (std_msgs/Int32)
Publishes the last valid count of AR tags

---Subscribed topics---
ar_pose_marker (ar_track_alvar/AlvarMarkers)
This is a list of the poses of all the observed AR tags, with respect to the output frame

---Services provided---
/reset (std_srvs/Empty)
Service used to manually reset the counter

---Parameter---
/ar_tag_number (int, default 9)
Number of possible detectable AR tags
/use_timer (bool, default true)
Enbale the automatic reset of the counter based on a timer
/reset_time (double, default 120.0);
Number of seconds between each automatic reset
