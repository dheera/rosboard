^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package web_video_server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.7 (2017-11-20)
------------------
* Ffmpeg 3 (`#43 <https://github.com/RobotWebTools/web_video_server/issues/43>`_)
  * Correct use of deprecated parameters
  codec_context\_->rc_buffer_aggressivity marked as "currently useless", so removed
  codec_context\_->frame_skip_threshold access through new priv_data api
  * New names for pixel formats
  * AVPicture is deprecated, use AVFrame
  * Switch to non-deprecated free functions
  * Use new encoding api for newer versions
  * codec_context is deprecated, use packet flags
* Update travis configuration to test against kinetic (`#44 <https://github.com/RobotWebTools/web_video_server/issues/44>`_)
* fixed misuse of remove_if (`#35 <https://github.com/RobotWebTools/web_video_server/issues/35>`_)
* Merge pull request `#33 <https://github.com/RobotWebTools/web_video_server/issues/33>`_ from achim-k/patch-1
  web_video_server: fix bool function not returning
  This fix is required when compiling the package with `clang`. Otherwise a SIGILL (Illegal instruction) is triggered.
* Contributors: Hans-Joachim Krauch, Jan, Jihoon Lee, russelhowe

0.0.6 (2017-01-17)
------------------
* Fixed topic list to display all image topics, fixing Issue `#18 <https://github.com/RobotWebTools/web_video_server/issues/18>`_.
* Contributors: Eric

0.0.5 (2016-10-13)
------------------
* Merge pull request `#23 <https://github.com/RobotWebTools/web_video_server/issues/23>`_ from iki-wgt/develop
  More information when server creation is failed
* Removed empty line
* More detailed exception message
  Programm behavior is not changed since the exception is rethrown.
* Contributors: BennyRe, Russell Toris

0.0.4 (2015-08-18)
------------------
* Merge pull request #16 from mitchellwills/compressed
  Adds support for streaming ROS compressed image topics without the need to reencode them
* Switch to checkout async_web_server_cpp from source
* Upgrade for change in signature of async_web_server_cpp request handler
* Added ros compressed video streamer type
  This directly passes the ros compressed frame data to the http socket without reencoding it
* Switched from passing image transport to passing node handle to streamer constructors
* Added default transport parameter for regular image streamers
* Contributors: Mitchell Wills, Russell Toris

0.0.3 (2015-05-07)
------------------
* added verbose flag
* Contributors: Russell Toris

0.0.2 (2015-02-20)
------------------
* Merge pull request #10 from mitchellwills/develop
  Added option to specify server address
* Added option to specify server address
* Merge pull request #3 from mitchellwills/develop
  Remove old http_server implementation and replace it with async_web_server_cpp package
* Moved from using built in http server to new async_web_server_cpp package
* Did some cleanup of streamers
* Update package.xml
* Contributors: Mitchell Wills, Russell Toris

0.0.1 (2014-10-30)
------------------
* missed travis file
* cleanup and travis build
* ROS auto-format
* Merge pull request #1 from mitchellwills/develop
  Initial implementation of a http web server that serves ROS image topics as multiple web compatible formats
* Made some changes suggested by catkin_lint and did some package cleanup
* Added support for libavstreamer on Ubuntu 13.10 version of libav
* Added support for specifying vp8 quality parameter
* Implemented lazy initialization for libav buffers so that output size can be infered from the first image
* Updated README
* Added vp8 support
* Broke image encodings out into different files
* Made write operations async
  Send timestamps for mjpeg stream
* Initial commit
* Update README.md
* Update README.md
* Update README.md
* Initial commit
* Contributors: Mitchell Wills, Russell Toris
