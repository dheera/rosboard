^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package async_web_server_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2015-08-18)
------------------
* Merge pull request #6 from mitchellwills/develop
  Added some tests
* Added filesystem tests
* Added some more tests (including websocket tests)
* Added some more http tests
* Added an echo test and enabled tests on travis
* Began working on some tests
* Fixed missing boost dep
* Merge pull request #5 from mitchellwills/develop
  Some more improvements
* Added filesystem request handler
  This serves both files from a given root and directory listings (if requested)
* Modified request handler signature so that it can reject requests (causing them to be pushed to the next handler in a handler group)
* Now load a file to be served each time it is requested
* Fix HTTP Server stop to allow it to be safe to call multiple times
* Merge pull request #4 from mitchellwills/develop
  A few small improvements
* Allow for a server to be created even if the system has no non-local IP
  See http://stackoverflow.com/questions/5971242/how-does-boost-asios-hostname-resolution-work-on-linux-is-it-possible-to-use-n
* Changed write resource to be a const shared ptr
* Contributors: Mitchell Wills, Russell Toris

0.0.2 (2015-01-06)
------------------
* Merge pull request #3 from mitchellwills/develop
  Added support for specifying additional headers when creating static request handlers
* Added some comments to the HTTP reply methods
* Added support for specifying additional headers when creating static request handlers
* Contributors: Mitchell Wills, Russell Toris

0.0.1 (2014-12-02)
------------------
* OCD clenup
* Merge pull request #2 from mitchellwills/develop
  Few small fixes
* Fixed message processing so close message is actually passed through
* Fixed potential memory leak from boost shared_ptr misuse
* Merge pull request #1 from mitchellwills/develop
  Import of initial implementation from webrtc_ros repo
* Fixed install directives
* Added some documentation
* Added more metadata files
* Cleaned up file formatting
* Fixed compilation error on 12.04
* Added boost dependancy
* Added travis configuration
* Package cleanup
* renamed package to async_web_server_cpp
* Initial import of cpp_web_server from webrtc_ros
* Initial commit
* Contributors: Mitchell Wills, Russell Toris
