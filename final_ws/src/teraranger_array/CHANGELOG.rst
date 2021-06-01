^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package teraranger_array
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2019-05-03)
------------------
* Update package description
* Merge pull request `#60 <https://github.com/Terabee/teraranger_array/issues/60>`_ from FRC900/remove_lib_from_cmake
  Removed teraranger_array library from catkin_package LIBRARIES
* Merge remote-tracking branch 'origin/master' into remove_lib_from_cmake
* Merge pull request `#59 <https://github.com/Terabee/teraranger_array/issues/59>`_ from FRC900/cpp_msg_header
  Move header to start of RangeArray.msg
* Add travis config
* Removed teraranger_array library from catkin_package LIBRARIES
  This file isn't built, so packages referencing teraranger_array
  fail when searching for the non-existent library
* Move header to start of RangeArray.msg
  The cpp tools require the header to be the first entry in the
  message for ros::message_traits to recognize that the message
  in fact has a header.
* Update links in Readme
* Contributors: Kevin Jaget, Pierre-Louis Kabaradjian

1.3.4 (2018-10-08)
------------------
* Update maintainer list
* Fix reconfigure infos
* Make sure processAck always returns something
* Mention dependency on serial package in readme
* Avoid narrowing cast in switch case
* Contributors: Morten Fyhn Amundsen, Pierre-Louis Kabaradjian

1.3.3 (2018-09-07)
------------------
* Update Readme for Evo 3m
* Add Evo 3m type to the list of available Evo's
* Contributors: BaptistePotier, Pierre-Louis Kabaradjian

1.3.2 (2018-07-25)
------------------
* Update Readme for Tower Evo
* Set default rate to ASAP
* Add 500 et 600 Hz mode
* Add specific firing mode for TeraRanger Tower Evo
* Contributors: Baptiste Potier, Pierre-Louis Kabaradjian

1.3.1 (2018-05-22)
------------------
* Change default modes
* Re-add line removed by mistake
* Correct typo in example launch files
* Contributors: Pierre-Louis Kabaradjian

1.3.0 (2018-04-13)
------------------
* Add example launch files
* Update for evo 600hz
* Close serial port on shutdown
* Move input flush
* Fix/default modes
* Make separate function for each reconfigure parameter
* Initilalize all modes at first dynamic reconfigure call
* Remove min and max clipping for One and Evo
* Contributors: Pierre-Louis Kabaradjian, Baptiste Potier

1.2.3 (2017-12-08)
------------------
* Correct linear acceleration conversion factor to a more accurate one
* Contributors: Pierre-Louis Kabaradjian

1.2.2 (2017-12-07)
------------------
* Reduce queue sizes to 1
* Add separate topic for euler imu data
* Contributors: Pierre-Louis Kabaradjian

1.2.1 (2017-12-06)
------------------
* Correct wrong euler factor
* Contributors: Pierre-Louis Kabaradjian

1.2.0 (2017-12-05)
------------------
* Clean dynamic reconfigure .cfg files
* Remove unsupported modes
* Add node namespace to every log message
* Remove old debug messages
* Add ack check when sending commands
* Correct rate commands
* Set defaults modes in both dynamic reconfigure and driver init
* Disable custom firing mode
* Remove unnecessary rates
* Contributors: Pierre-Louis Kabaradjian

1.1.0 (2017-11-17)
------------------
* Change license to MIT
* Update link
* Contributors: Pierre-Louis Kabaradjian, Baptiste Potier

1.0.1 (2017-09-20)
------------------
* Update package.xml
* Contributors: Pierre-Louis Kabaradjian

1.0.0 (2017-09-18)
------------------

* Use ros-serial and remove old serial files
* Standardize topic names
* Use REP 117
* Use RangeArray message, append namespace to frame_id
* Send disable cmd when driver exits
* Refactor trone and multiflex drivers
* Initial commit

* Contributors: Pierre-Louis Kabaradjian, Krzysztof Zurad, Mateusz Sadowski, Baptiste Potier
