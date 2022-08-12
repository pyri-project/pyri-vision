from pyri.sandbox import blockly_compiler
import io

def _do_blockly_compile_test(blockly_json, expected_pysrc):
    json_io = io.StringIO(blockly_json)
    output_io = io.StringIO()

    blockly_compiler.compile_blockly_file(json_io, output_io)
    output_io.seek(0)
    pysrc_str = output_io.read()
    print(pysrc_str)
    assert pysrc_str == expected_pysrc

def test_blockly_compiler_vision():
    vision_blockly_json = \
"""
{
  "blocks": {
    "languageVersion": 0,
    "blocks": [
      {
        "type": "procedures_defnoreturn",
        "id": "aaaaa",
        "x": 20,
        "y": 20,
        "icons": {
          "comment": {
            "text": "Describe this function...",
            "pinned": false,
            "height": 80,
            "width": 160
          }
        },
        "fields": {
          "NAME": "vision_blocks_test"
        },
        "inputs": {
          "STACK": {
            "block": {
              "type": "variables_set",
              "id": "o7WhM+.aYMc?fImv/i{O",
              "fields": {
                "VAR": {
                  "id": "xT0{e~/[6L+IAHy~O%1p"
                }
              },
              "inputs": {
                "VALUE": {
                  "block": {
                    "type": "vision_detect_aruco",
                    "id": "I~DxyRnIqmsZz5gEdQ-4",
                    "fields": {
                      "ARUCO_DICT": "DICT_4X4_100"
                    },
                    "inputs": {
                      "ARUCO_ID": {
                        "block": {
                          "type": "variables_get",
                          "id": "5LBQ2MpecGGN1eZ$*HPm",
                          "fields": {
                            "VAR": {
                              "id": "pj$s;RcVLObOt:pVb]W9"
                            }
                          }
                        }
                      },
                      "MARKER_SIZE": {
                        "block": {
                          "type": "variables_get",
                          "id": "r6E0aeT;OXKTV1[r7Mmp",
                          "fields": {
                            "VAR": {
                              "id": "$PPx(IynLka8~c]S(rN;"
                            }
                          }
                        }
                      },
                      "ROI": {
                        "block": {
                          "type": "variables_get",
                          "id": "^z(z:|kinwCw0q`X-yn$",
                          "fields": {
                            "VAR": {
                              "id": "+V7AVr%a(1|4_Cua?:X*"
                            }
                          }
                        }
                      }
                    }
                  }
                }
              },
              "next": {
                "block": {
                  "type": "math_change",
                  "id": "uJv9OSN%y0@S;2-3g-o1",
                  "fields": {
                    "VAR": {
                      "id": "+V7AVr%a(1|4_Cua?:X*"
                    }
                  },
                  "inputs": {
                    "DELTA": {
                      "shadow": {
                        "type": "math_number",
                        "id": "6oI[dz|@9c7pR[q:+@W1",
                        "fields": {
                          "NUM": 1
                        }
                      },
                      "block": {
                        "type": "vision_aruco_detected_get_pose",
                        "id": "JmwR!HN!0DBGL(rJsmBz",
                        "inputs": {
                          "DETECTED_MARKER": {
                            "block": {
                              "type": "variables_get",
                              "id": "zJIPAScW,J(1^RpP:qnF",
                              "fields": {
                                "VAR": {
                                  "id": "$PPx(IynLka8~c]S(rN;"
                                }
                              }
                            }
                          }
                        }
                      }
                    }
                  },
                  "next": {
                    "block": {
                      "type": "variables_set",
                      "id": "uIMK@axPW5/cgu~YiDTZ",
                      "fields": {
                        "VAR": {
                          "id": "$PPx(IynLka8~c]S(rN;"
                        }
                      },
                      "inputs": {
                        "VALUE": {
                          "block": {
                            "type": "vision_aruco_detected_get_id",
                            "id": "2;}=?;@YDC?+tcy*09AF",
                            "inputs": {
                              "DETECTED_MARKER": {
                                "block": {
                                  "type": "variables_get",
                                  "id": "1QjVL(cZ.d{er`h8ivu*",
                                  "fields": {
                                    "VAR": {
                                      "id": "pj$s;RcVLObOt:pVb]W9"
                                    }
                                  }
                                }
                              }
                            }
                          }
                        }
                      },
                      "next": {
                        "block": {
                          "type": "variables_set",
                          "id": "G+)8(G}z~9r}r$ng{r.s",
                          "fields": {
                            "VAR": {
                              "id": "pj$s;RcVLObOt:pVb]W9"
                            }
                          },
                          "inputs": {
                            "VALUE": {
                              "block": {
                                "type": "vision_template_match",
                                "id": "n|%1gU~1KB3@x]]}/@}a",
                                "inputs": {
                                  "TEMPLATE": {
                                    "block": {
                                      "type": "variables_get",
                                      "id": "~7TzV88g7#N3mmG;{B0Z",
                                      "fields": {
                                        "VAR": {
                                          "id": "+V7AVr%a(1|4_Cua?:X*"
                                        }
                                      }
                                    }
                                  },
                                  "OBJECT_Z": {
                                    "block": {
                                      "type": "variables_get",
                                      "id": "kh1h6|ge-IUg16(Z28m/",
                                      "fields": {
                                        "VAR": {
                                          "id": "xT0{e~/[6L+IAHy~O%1p"
                                        }
                                      }
                                    }
                                  },
                                  "ROI": {
                                    "block": {
                                      "type": "variables_get",
                                      "id": "fkh]cJOD48e3zkw`eU2:",
                                      "fields": {
                                        "VAR": {
                                          "id": "$PPx(IynLka8~c]S(rN;"
                                        }
                                      }
                                    }
                                  }
                                }
                              }
                            }
                          },
                          "next": {
                            "block": {
                              "type": "variables_set",
                              "id": "sPZ]2k7KXR63n@nYn=9#",
                              "fields": {
                                "VAR": {
                                  "id": "xT0{e~/[6L+IAHy~O%1p"
                                }
                              },
                              "inputs": {
                                "VALUE": {
                                  "block": {
                                    "type": "vision_matched_template_get_pose",
                                    "id": "14@dZ)MXj,JLrXNm2t]O",
                                    "inputs": {
                                      "TEMPLATE_MATCH": {
                                        "block": {
                                          "type": "variables_get",
                                          "id": "l{!c5g5aCht-1BT5pUDD",
                                          "fields": {
                                            "VAR": {
                                              "id": "+V7AVr%a(1|4_Cua?:X*"
                                            }
                                          }
                                        }
                                      }
                                    }
                                  }
                                }
                              }
                            }
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    ]
  },
  "variables": [
    {
      "name": "var1",
      "id": "xT0{e~/[6L+IAHy~O%1p"
    },
    {
      "name": "var4",
      "id": "+V7AVr%a(1|4_Cua?:X*"
    },
    {
      "name": "var2",
      "id": "pj$s;RcVLObOt:pVb]W9"
    },
    {
      "name": "var3",
      "id": "$PPx(IynLka8~c]S(rN;"
    }
  ]
}
"""
    expected_pysrc = \
        "# Describe this function...\n" \
        "def vision_blocks_test():\n" \
        "\n" \
        "  var1 = vision_detect_aruco(\"DICT_4X4_100\", var2, var3, var4)\n" \
        "  var4 = (var4 if isinstance(var4, Number) else 0) + (vision_aruco_detected_get_pose(var3))\n" \
        "  var3 = vision_aruco_detected_get_id(var2)\n" \
        "  var2 = vision_template_match(var4, var1, var3)\n" \
        "  var1 = vision_matched_template_get_pose(var4)\n"

    _do_blockly_compile_test(vision_blockly_json, expected_pysrc)