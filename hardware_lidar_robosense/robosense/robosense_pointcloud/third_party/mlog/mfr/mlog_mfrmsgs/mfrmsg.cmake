file(GLOB_RECURSE MFRMSGS ${CMAKE_CURRENT_LIST_DIR}/*.mfrmsg)
generate_mfrmessage(
  PACKAGENAME
    mlog_mfrmsgs
  MSGFILES
    ${MFRMSGS}
  DEPENDENCIES
    std_mfrmsgs
)