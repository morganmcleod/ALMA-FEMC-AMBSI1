_disconnect
_connect
IF BUSCON0 = 4AE
_einit
_clearselections
_addselection FA60,FBFF
_copy
_paste 00000
_download
_compare
_paste 00200
_download
_compare
_paste 00002
_download
_compare
_paste 00400
_download
_compare
_paste 00004
_download
_compare
_paste 00800
_download
_compare
_paste 00008
_download
_compare
_paste 01000
_download
_compare
_paste 00010
_download
_compare
_paste 02000
_download
_compare
_paste 00020
_download
_compare
_paste 04000
_download
_compare
_paste 00040
_download
_compare
_paste 08000
_download
_compare
_paste 00080
_download
_compare
_paste 10000
_download
_compare
_paste 00100
_download
_compare
_paste 20000
_download
_compare
_movemonitor 00000
_movemonitor 00200
_movemonitor 00002
_movemonitor 00400
_movemonitor 00004
_movemonitor 00800
_movemonitor 00008
_movemonitor 01000
_movemonitor 00010
_movemonitor 02000
_movemonitor 00020
_movemonitor 04000
_movemonitor 00040
_movemonitor 08000
_movemonitor 00080
_movemonitor 10000
_movemonitor 00100
_movemonitor 20000
ELSE
_message "Please load test_ambsi.ini preferences and start the script again."
ENDIF
_clearselections
_disconnect
