# Generic AEDAT packet message used as container for event types that do not have a translation to ROS yet. Some information extracted (duplicated) to make it accessible to ROS

Header header

uint32 aedat_type # AEDAT type identifier extracted from package
uint16 event_source # AEDAT source identifier extracted from package

byte[] data # The actual AEDAT package: Full, unaltered package, the bit-encoding is according to the 3.1 aedat format specification found on our support wiki