_disconnect
_connect
IF BUSCON0 = 48C
_einit
_clearselections
_addselection FA60,FBFF
_copy
_paste 00000
_iprogram
_compare
_paste 00200
_iprogram
_compare
_paste 00002
_iprogram
_compare
_paste 00400
_iprogram
_compare
_paste 00004
_iprogram
_compare
_paste 00800
_iprogram
_compare
_paste 00008
_iprogram
_compare
_paste 01000
_iprogram
_compare
_paste 00010
_iprogram
_compare
_paste 02000
_iprogram
_compare
_paste 00020
_iprogram
_compare
_paste 04000
_iprogram
_compare
_paste 00040
_iprogram
_compare
_paste 08000
_iprogram
_compare
_paste 00080
_iprogram
_compare
_paste 10000
_iprogram
_compare
_paste 00100
_iprogram
_compare
_paste 20000
_iprogram
_compare
_erase PROGRAMFLASH,1
ELSE
_message "Please load flash_ambsi.ini preferences and start the script again."
ENDIF
_clearselections
_disconnect
