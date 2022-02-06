					
set(SRC_NEW
  DTSU666H_translator.cpp
  ModbusMessage.cpp
  Logging.cpp
  ModbusClient.cpp
  ModbusClientRTU.cpp
  ModbusServer.cpp
  ModbusServerRTU.cpp
  RTUutils.cpp 
  )

set(srcs ${SRC_NEW} )

idf_component_register(SRCS ${srcs} INCLUDE_DIRS "include" )


