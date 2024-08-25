file(GLOB_RECURSE MFRMSGS ${CMAKE_CURRENT_LIST_DIR}/*.mfrmsg)
generate_mfrmessage(
  PACKAGENAME
    std_mfrmsgs
  MSGFILES
    ${MFRMSGS}
)