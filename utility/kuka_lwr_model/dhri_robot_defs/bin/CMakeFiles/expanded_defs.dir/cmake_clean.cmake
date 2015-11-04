FILE(REMOVE_RECURSE
  "CMakeFiles/expanded_defs"
  "../robots/kuka_lwr_arm.expanded.xml"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/expanded_defs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
