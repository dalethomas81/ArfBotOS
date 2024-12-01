( https://content.helpme-codesys.com/en/CODESYS%20SoftMotion/_sm_cnc_din66025_basics.html) 
( https://product-help.schneider-electric.com/Machine%20Expert/V2.0/en/codesys_softmotion/codesys_softmotion/modules/_sm_cnc_din66025_coordinate_shift.html) 
( https://content.helpme-codesys.com/en/CODESYS%20SoftMotion/_sm_cnc_din66025_change_variable_values.htmlhttps://content.helpme-codesys.com/en/CODESYS%20SoftMotion/_sm_cnc_din66025_change_variable_values.html) 
( G00 rapid positioning | G01 positioning at a feedrate) 
( G02 circular clockwise | G03 circular counter clockwise) 
( G04 dwell) 
( G17 xy plane | G18 xz plane | G19 yz plane) 
( G20 inch | G21 mm) 
( G43 tool length offset | G49 cancel tool length offset) 
( G90 absolute G91 incremental | G92 offset coordinate system) 
( https://content.helpme-codesys.com/en/CODESYS%20SoftMotion/_sm_cnc_din66025_coordinate_shift.html )
( G53 reset decoder position | G54 absolute offset | G55 relative offset | G56 set offset to current )
(##########)
(Hello, World.)
(##########)
N0 M300 (set output 0)
N10 G56 X0 Y0 Z0 A0 B-90 C0 (set current as start)
(When using Fusion for Personal Use, the feedrate of rapid)
(moves is reduced to match the feedrate of cutting moves,)
(which can increase machining time. Unrestricted rapid moves)
(are available with a Fusion Subscription.)
N30 G1 X-1.132 Y-76.713 Z10 F30
N40  Z5
N50  Z0
N60  X8.295 Y-76.692
N70  X8.3 Y-79.338
N80  X-11.7 Y-79.383
N90  X-11.705 Y-76.737
N100  X-3.493 Y-76.718
N110  X-3.516 Y-66.323
N120  X-11.729 Y-66.341
N130  X-11.735 Y-63.694
N140  X8.265 Y-63.65
N150  X8.271 Y-66.296
N160  X-1.156 Y-66.317
N170  X-1.132 Y-76.713
N180  Z5
N190  Z6
N200  X0.248 Y-60.364
N210  Z5
N220  Z0
N230  X-0.177 Y-60.328
N240  X-0.59 Y-60.278
N250  X-0.989 Y-60.212
N260  X-1.376 Y-60.132
N270  X-1.75 Y-60.037
N280  X-2.112 Y-59.928
N290  X-2.46 Y-59.804
N300  X-2.796 Y-59.665
N310  X-3.119 Y-59.511
N320  X-3.429 Y-59.342
N330  X-3.727 Y-59.159
N340  X-4.012 Y-58.961
N350  X-4.284 Y-58.749
N360  X-4.543 Y-58.521
N370  X-4.788 Y-58.282
N380  X-5.017 Y-58.034
N390  X-5.231 Y-57.778
N400  X-5.429 Y-57.512
N410  X-5.611 Y-57.238
N420  X-5.777 Y-56.955
N430  X-5.927 Y-56.663
N440  X-6.062 Y-56.362
N450  X-6.181 Y-56.053
N460  X-6.284 Y-55.735
N470  X-6.372 Y-55.408
N480  X-6.444 Y-55.072
N490  X-6.5 Y-54.728
N500  X-6.54 Y-54.375
N510  X-6.564 Y-54.013
N520  X-6.573 Y-53.642
N530  X-6.566 Y-53.283
N540  X-6.544 Y-52.932
N550  X-6.506 Y-52.589
N560  X-6.453 Y-52.255
N570  X-6.384 Y-51.929
N580  X-6.299 Y-51.611
N590  X-6.2 Y-51.302
N600  X-6.084 Y-51.001
N610  X-5.954 Y-50.708
N620  X-5.808 Y-50.424
N630  X-5.646 Y-50.147
N640  X-5.469 Y-49.879
N650  X-5.276 Y-49.619
N660  X-5.068 Y-49.368
N670  X-4.845 Y-49.125
N680  X-4.606 Y-48.89
N690  X-4.353 Y-48.666
N700  X-4.087 Y-48.457
N710  X-3.809 Y-48.262
N720  X-3.518 Y-48.082
N730  X-3.214 Y-47.915
N740  X-2.898 Y-47.764
N750  X-2.569 Y-47.626
N760  X-2.228 Y-47.503
N770  X-1.874 Y-47.394
N780  X-1.508 Y-47.3
N790  X-1.128 Y-47.22
N800  X-0.737 Y-47.154
N810  X-0.332 Y-47.103
N820  X0.085 Y-47.066
N830  X0.514 Y-47.044
N840  X0.956 Y-47.036
N850  X1.079
N860  X1.229 Y-47.038
N870  X1.407 Y-47.042
N880  X1.611 Y-47.048
N890  X1.636 Y-57.853
N900  X1.93 Y-57.831
N910  X2.215 Y-57.8
N920  X2.491 Y-57.762
N930  X2.759 Y-57.715
N940  X3.018 Y-57.659
N950  X3.268 Y-57.595
N960  X3.51 Y-57.523
N970  X3.742 Y-57.442
N980  X3.966 Y-57.353
N990  X4.182 Y-57.255
N1000  X4.388 Y-57.149
N1010  X4.586 Y-57.035
N1020  X4.775 Y-56.912
N1030  X4.955 Y-56.781
N1040  X5.126 Y-56.642
N1050  X5.289 Y-56.494
N1060  X5.442 Y-56.339
N1070  X5.585 Y-56.18
N1080  X5.719 Y-56.016
N1090  X5.842 Y-55.848
N1100  X5.956 Y-55.675
N1110  X6.06 Y-55.497
N1120  X6.153 Y-55.314
N1130  X6.237 Y-55.127
N1140  X6.311 Y-54.936
N1150  X6.375 Y-54.739
N1160  X6.429 Y-54.538
N1170  X6.473 Y-54.332
N1180  X6.508 Y-54.122
N1190  X6.532 Y-53.907
N1200  X6.546 Y-53.687
N1210  X6.551 Y-53.462
N1220  X6.539 Y-53.131
N1230  X6.505 Y-52.812
N1240  X6.449 Y-52.505
N1250  X6.371 Y-52.211
N1260  X6.27 Y-51.929
N1270  X6.148 Y-51.66
N1280  X6.003 Y-51.403
N1290  X5.836 Y-51.158
N1300  X5.645 Y-50.926
N1310  X5.428 Y-50.707
N1320  X5.184 Y-50.499
N1330  X4.913 Y-50.304
N1340  X4.617 Y-50.122
N1350  X4.293 Y-49.952
N1360  X3.944 Y-49.794
N1370  X3.568 Y-49.649
N1380  X3.876 Y-47.111
N1390  X4.15 Y-47.189
N1400  X4.417 Y-47.276
N1410  X4.676 Y-47.37
N1420  X4.927 Y-47.473
N1430  X5.17 Y-47.583
N1440  X5.405 Y-47.701
N1450  X5.633 Y-47.828
N1460  X5.853 Y-47.963
N1470  X6.065 Y-48.105
N1480  X6.269 Y-48.256
N1490  X6.466 Y-48.414
N1500  X6.655 Y-48.581
N1510  X6.836 Y-48.755
N1520  X7.009 Y-48.938
N1530  X7.175 Y-49.128
N1540  X7.333 Y-49.327
N1550  X7.482 Y-49.533
N1560  X7.621 Y-49.746
N1570  X7.751 Y-49.966
N1580  X7.872 Y-50.194
N1590  X7.983 Y-50.428
N1600  X8.084 Y-50.669
N1610  X8.176 Y-50.918
N1620  X8.258 Y-51.173
N1630  X8.33 Y-51.436
N1640  X8.393 Y-51.706
N1650  X8.447 Y-51.982
N1660  X8.49 Y-52.266
N1670  X8.525 Y-52.557
N1680  X8.549 Y-52.855
N1690  X8.564 Y-53.16
N1700  X8.57 Y-53.471
N1710  X8.563 Y-53.864
N1720  X8.541 Y-54.246
N1730  X8.504 Y-54.617
N1740  X8.451 Y-54.979
N1750  X8.383 Y-55.33
N1760  X8.299 Y-55.671
N1770  X8.201 Y-56.002
N1780  X8.087 Y-56.322
N1790  X7.957 Y-56.632
N1800  X7.813 Y-56.932
N1810  X7.653 Y-57.222
N1820  X7.478 Y-57.501
N1830  X7.287 Y-57.771
N1840  X7.081 Y-58.03
N1850  X6.86 Y-58.278
N1860  X6.623 Y-58.517
N1870  X6.373 Y-58.743
N1880  X6.11 Y-58.954
N1890  X5.835 Y-59.151
N1900  X5.548 Y-59.334
N1910  X5.248 Y-59.502
N1920  X4.936 Y-59.655
N1930  X4.612 Y-59.794
N1940  X4.275 Y-59.919
N1950  X3.926 Y-60.029
N1960  X3.564 Y-60.124
N1970  X3.191 Y-60.205
N1980  X2.804 Y-60.271
N1990  X2.406 Y-60.323
N2000  X1.995 Y-60.36
N2010  X1.572 Y-60.383
N2020  X1.136 Y-60.391
N2030  X0.686 Y-60.385
N2040  X0.248 Y-60.364
N2050  Z5
N2060  Z6
N2070  X-0.384 Y-57.721
N2080  Z5
N2090  Z0
N2100  X-0.402 Y-49.631
N2110  X-0.845 Y-49.682
N2120  X-1.259 Y-49.752
N2130  X-1.644 Y-49.841
N2140  X-2.001 Y-49.948
N2150  X-2.329 Y-50.074
N2160  X-2.629 Y-50.219
N2170  X-2.9 Y-50.382
N2180  X-3.142 Y-50.565
N2190  X-3.314 Y-50.714
N2200  X-3.474 Y-50.869
N2210  X-3.623 Y-51.03
N2220  X-3.761 Y-51.196
N2230  X-3.888 Y-51.368
N2240  X-4.004 Y-51.544
N2250  X-4.109 Y-51.727
N2260  X-4.203 Y-51.914
N2270  X-4.286 Y-52.107
N2280  X-4.357 Y-52.306
N2290  X-4.418 Y-52.509
N2300  X-4.467 Y-52.719
N2310  X-4.506 Y-52.933
N2320  X-4.533 Y-53.154
N2330  X-4.549 Y-53.379
N2340  X-4.554 Y-53.61
N2350  X-4.535 Y-54.025
N2360  X-4.481 Y-54.422
N2370  X-4.392 Y-54.803
N2380  X-4.267 Y-55.166
N2390  X-4.107 Y-55.513
N2400  X-3.912 Y-55.843
N2410  X-3.681 Y-56.156
N2420  X-3.415 Y-56.452
N2430  X-3.271 Y-56.592
N2440  X-3.12 Y-56.723
N2450  X-2.963 Y-56.847
N2460  X-2.8 Y-56.963
N2470  X-2.632 Y-57.07
N2480  X-2.457 Y-57.17
N2490  X-2.277 Y-57.261
N2500  X-2.09 Y-57.344
N2510  X-1.898 Y-57.419
N2520  X-1.699 Y-57.487
N2530  X-1.495 Y-57.546
N2540  X-1.285 Y-57.597
N2550  X-1.068 Y-57.64
N2560  X-0.846 Y-57.675
N2570  X-0.618 Y-57.702
N2580  X-0.384 Y-57.721
N2590  Z5
N2600  Z6
N2610  X-0.35 Y-44.092
N2620  Z5
N2630  Z0
N2640  X-11.779 Y-44.117
N2650  X-11.784 Y-41.662
N2660  X8.216 Y-41.617
N2670  X8.221 Y-44.072
N2680  X-0.35 Y-44.092
N2690  Z5
N2700  Z6
N2710  X8.207 Y-37.865
N2720  Z5
N2730  Z0
N2740  X-11.793 Y-37.91
N2750  X-11.798 Y-35.454
N2760  X8.202 Y-35.409
N2770  X8.207 Y-37.865
N2780  Z5
N2790  Z6
N2800  X5.183 Y-31.637
N2810  Z5
N2820  Z0
N2830  X4.865 Y-31.792
N2840  X4.535 Y-31.932
N2850  X4.19 Y-32.057
N2860  X3.832 Y-32.168
N2870  X3.461 Y-32.264
N2880  X3.077 Y-32.346
N2890  X2.678 Y-32.412
N2900  X2.267 Y-32.465
N2910  X1.842 Y-32.502
N2920  X1.403 Y-32.525
N2930  X0.951 Y-32.534
N2940  X0.456 Y-32.526
N2950  X-0.022 Y-32.501
N2960  X-0.485 Y-32.458
N2970  X-0.931 Y-32.398
N2980  X-1.361 Y-32.32
N2990  X-1.774 Y-32.225
N3000  X-2.172 Y-32.112
N3010  X-2.553 Y-31.982
N3020  X-2.918 Y-31.834
N3030  X-3.266 Y-31.669
N3040  X-3.598 Y-31.486
N3050  X-3.914 Y-31.286
N3060  X-4.214 Y-31.068
N3070  X-4.498 Y-30.833
N3080  X-4.765 Y-30.58
N3090  X-5.016 Y-30.309
N3100  X-5.211 Y-30.073
N3110  X-5.394 Y-29.83
N3120  X-5.564 Y-29.581
N3130  X-5.722 Y-29.325
N3140  X-5.867 Y-29.063
N3150  X-6 Y-28.795
N3160  X-6.12 Y-28.52
N3170  X-6.228 Y-28.239
N3180  X-6.323 Y-27.951
N3190  X-6.405 Y-27.657
N3200  X-6.475 Y-27.356
N3210  X-6.532 Y-27.049
N3220  X-6.577 Y-26.735
N3230  X-6.609 Y-26.416
N3240  X-6.629 Y-26.089
N3250  X-6.636 Y-25.757
N3260  X-6.629 Y-25.387
N3270  X-6.607 Y-25.027
N3280  X-6.569 Y-24.674
N3290  X-6.517 Y-24.331
N3300  X-6.448 Y-23.995
N3310  X-6.365 Y-23.669
N3320  X-6.266 Y-23.35
N3330  X-6.152 Y-23.041
N3340  X-6.023 Y-22.739
N3350  X-5.878 Y-22.447
N3360  X-5.718 Y-22.162
N3370  X-5.543 Y-21.886
N3380  X-5.353 Y-21.619
N3390  X-5.147 Y-21.36
N3400  X-4.925 Y-21.11
N3410  X-4.689 Y-20.868
N3420  X-4.439 Y-20.638
N3430  X-4.177 Y-20.423
N3440  X-3.904 Y-20.222
N3450  X-3.619 Y-20.036
N3460  X-3.322 Y-19.865
N3470  X-3.013 Y-19.709
N3480  X-2.693 Y-19.567
N3490  X-2.361 Y-19.441
N3500  X-2.017 Y-19.329
N3510  X-1.662 Y-19.232
N3520  X-1.295 Y-19.149
N3530  X-0.916 Y-19.082
N3540  X-0.526 Y-19.029
N3550  X-0.123 Y-18.991
N3560  X0.29 Y-18.968
N3570  X0.716 Y-18.96
N3580  X1.061 Y-18.962
N3590  X1.397 Y-18.971
N3600  X1.723 Y-18.987
N3610  X2.04 Y-19.009
N3620  X2.348 Y-19.038
N3630  X2.646 Y-19.073
N3640  X2.935 Y-19.115
N3650  X3.215 Y-19.164
N3660  X3.485 Y-19.219
N3670  X3.746 Y-19.281
N3680  X3.998 Y-19.349
N3690  X4.241 Y-19.424
N3700  X4.474 Y-19.505
N3710  X4.698 Y-19.593
N3720  X4.912 Y-19.688
N3730  X5.118 Y-19.789
N3740  X5.508 Y-20.01
N3750  X5.875 Y-20.254
N3760  X6.221 Y-20.523
N3770  X6.544 Y-20.816
N3780  X6.845 Y-21.132
N3790  X7.123 Y-21.472
N3800  X7.379 Y-21.837
N3810  X7.613 Y-22.225
N3820  X7.822 Y-22.63
N3830  X8.003 Y-23.044
N3840  X8.156 Y-23.468
N3850  X8.282 Y-23.9
N3860  X8.38 Y-24.342
N3870  X8.45 Y-24.793
N3880  X8.493 Y-25.253
N3890  X8.508 Y-25.722
N3900  X8.501 Y-26.098
N3910  X8.479 Y-26.465
N3920  X8.441 Y-26.823
N3930  X8.389 Y-27.171
N3940  X8.321 Y-27.51
N3950  X8.238 Y-27.841
N3960  X8.14 Y-28.162
N3970  X8.026 Y-28.474
N3980  X7.897 Y-28.777
N3990  X7.753 Y-29.071
N4000  X7.594 Y-29.356
N4010  X7.419 Y-29.632
N4020  X7.229 Y-29.899
N4030  X7.024 Y-30.157
N4040  X6.803 Y-30.406
N4050  X6.568 Y-30.645
N4060  X6.318 Y-30.873
N4070  X6.054 Y-31.086
N4080  X5.777 Y-31.284
N4090  X5.487 Y-31.468
N4100  X5.183 Y-31.637
N4110  Z5
N4120  Z6
N4130  X5.111 Y-28.786
N4140  Z5
N4150  Z0
N4160  X5.278 Y-28.632
N4170  X5.434 Y-28.472
N4180  X5.58 Y-28.308
N4190  X5.715 Y-28.138
N4200  X5.839 Y-27.964
N4210  X5.952 Y-27.785
N4220  X6.054 Y-27.601
N4230  X6.146 Y-27.413
N4240  X6.227 Y-27.219
N4250  X6.297 Y-27.021
N4260  X6.356 Y-26.817
N4270  X6.404 Y-26.609
N4280  X6.441 Y-26.396
N4290  X6.468 Y-26.178
N4300  X6.484 Y-25.955
N4310  X6.488 Y-25.727
N4320  X6.482 Y-25.501
N4330  X6.466 Y-25.28
N4340  X6.438 Y-25.063
N4350  X6.399 Y-24.852
N4360  X6.35 Y-24.645
N4370  X6.29 Y-24.443
N4380  X6.219 Y-24.246
N4390  X6.137 Y-24.053
N4400  X6.044 Y-23.866
N4410  X5.94 Y-23.683
N4420  X5.826 Y-23.505
N4430  X5.7 Y-23.332
N4440  X5.564 Y-23.164
N4450  X5.417 Y-23
N4460  X5.259 Y-22.842
N4470  X5.09 Y-22.688
N4480  X4.91 Y-22.541
N4490  X4.719 Y-22.404
N4500  X4.516 Y-22.277
N4510  X4.302 Y-22.158
N4520  X4.076 Y-22.05
N4530  X3.839 Y-21.951
N4540  X3.591 Y-21.861
N4550  X3.332 Y-21.781
N4560  X3.06 Y-21.711
N4570  X2.778 Y-21.65
N4580  X2.484 Y-21.598
N4590  X2.179 Y-21.556
N4600  X1.862 Y-21.524
N4610  X1.535 Y-21.501
N4620  X1.195 Y-21.487
N4630  X0.844 Y-21.483
N4640  X0.514 Y-21.489
N4650  X0.193 Y-21.504
N4660  X-0.117 Y-21.528
N4670  X-0.418 Y-21.562
N4680  X-0.708 Y-21.606
N4690  X-0.988 Y-21.659
N4700  X-1.257 Y-21.722
N4710  X-1.517 Y-21.794
N4720  X-1.766 Y-21.875
N4730  X-2.005 Y-21.967
N4740  X-2.234 Y-22.067
N4750  X-2.452 Y-22.177
N4760  X-2.661 Y-22.297
N4770  X-2.859 Y-22.426
N4780  X-3.047 Y-22.565
N4790  X-3.225 Y-22.713
N4800  X-3.392 Y-22.869
N4810  X-3.549 Y-23.029
N4820  X-3.695 Y-23.193
N4830  X-3.829 Y-23.362
N4840  X-3.953 Y-23.536
N4850  X-4.067 Y-23.715
N4860  X-4.169 Y-23.897
N4870  X-4.261 Y-24.085
N4880  X-4.341 Y-24.277
N4890  X-4.411 Y-24.474
N4900  X-4.47 Y-24.676
N4910  X-4.518 Y-24.882
N4920  X-4.556 Y-25.092
N4930  X-4.582 Y-25.308
N4940  X-4.598 Y-25.527
N4950  X-4.603 Y-25.752
N4960  X-4.597 Y-25.98
N4970  X-4.58 Y-26.202
N4980  X-4.553 Y-26.42
N4990  X-4.515 Y-26.633
N5000  X-4.466 Y-26.841
N5010  X-4.406 Y-27.045
N5020  X-4.336 Y-27.243
N5030  X-4.255 Y-27.436
N5040  X-4.163 Y-27.624
N5050  X-4.06 Y-27.808
N5060  X-3.947 Y-27.986
N5070  X-3.822 Y-28.16
N5080  X-3.688 Y-28.328
N5090  X-3.542 Y-28.492
N5100  X-3.385 Y-28.651
N5110  X-3.218 Y-28.805
N5120  X-3.04 Y-28.951
N5130  X-2.851 Y-29.088
N5140  X-2.651 Y-29.216
N5150  X-2.44 Y-29.334
N5160  X-2.218 Y-29.443
N5170  X-1.986 Y-29.542
N5180  X-1.742 Y-29.632
N5190  X-1.487 Y-29.712
N5200  X-1.221 Y-29.782
N5210  X-0.945 Y-29.843
N5220  X-0.657 Y-29.895
N5230  X-0.358 Y-29.937
N5240  X-0.049 Y-29.969
N5250  X0.272 Y-29.992
N5260  X0.603 Y-30.006
N5270  X0.945 Y-30.01
N5280  X1.288 Y-30.004
N5290  X1.619 Y-29.989
N5300  X1.94 Y-29.965
N5310  X2.249 Y-29.931
N5320  X2.548 Y-29.888
N5330  X2.836 Y-29.835
N5340  X3.113 Y-29.772
N5350  X3.378 Y-29.701
N5360  X3.633 Y-29.619
N5370  X3.877 Y-29.529
N5380  X4.11 Y-29.429
N5390  X4.332 Y-29.319
N5400  X4.543 Y-29.2
N5410  X4.743 Y-29.072
N5420  X4.932 Y-28.934
N5430  X5.111 Y-28.786
N5440  Z5
N5450  Z6
N5460  X5.36 Y-15.429
N5470  Z5
N5480  Z0
N5490  X5.354 Y-12.633
N5500  X8.151 Y-12.626
N5510  X8.527 Y-12.634
N5520  X8.884 Y-12.659
N5530  X9.224 Y-12.701
N5540  X9.544 Y-12.76
N5550  X9.846 Y-12.836
N5560  X10.13 Y-12.929
N5570  X10.395 Y-13.039
N5580  X10.642 Y-13.167
N5590  X10.872 Y-13.313
N5600  X11.09 Y-13.479
N5610  X11.293 Y-13.665
N5620  X11.484 Y-13.871
N5630  X11.661 Y-14.097
N5640  X11.825 Y-14.343
N5650  X11.975 Y-14.609
N5660  X12.112 Y-14.896
N5670  X11.063 Y-15.58
N5680  X10.972 Y-15.392
N5690  X10.872 Y-15.217
N5700  X10.761 Y-15.055
N5710  X10.641 Y-14.906
N5720  X10.51 Y-14.769
N5730  X10.369 Y-14.646
N5740  X10.219 Y-14.535
N5750  X10.058 Y-14.437
N5760  X9.883 Y-14.35
N5770  X9.69 Y-14.274
N5780  X9.48 Y-14.208
N5790  X9.251 Y-14.152
N5800  X9.004 Y-14.106
N5810  X8.738 Y-14.071
N5820  X8.455 Y-14.046
N5830  X8.154 Y-14.032
N5840  X8.157 Y-15.423
N5850  X5.36 Y-15.429
N5860  Z5
N5870  Z6
N5880  X2.404 Y1.738
N5890  Z5
N5900  Z0
N5910  X-11.873 Y-2.085
N5920  X-11.879 Y0.63
N5930  X1.225 Y3.702
N5940  X2.252 Y3.941
N5950  X3.277 Y4.163
N5960  X4.298 Y4.369
N5970  X5.315 Y4.557
N5980  X4.556 Y4.736
N5990  X3.881 Y4.896
N6000  X3.292 Y5.037
N6010  X3.029 Y5.1
N6020  X2.787 Y5.158
N6030  X2.566 Y5.212
N6040  X2.367 Y5.26
N6050  X2.189 Y5.304
N6060  X2.032 Y5.343
N6070  X1.896 Y5.376
N6080  X1.782 Y5.405
N6090  X1.688 Y5.43
N6100  X1.616 Y5.449
N6110  X-11.898 Y9.225
N6120  X-11.906 Y12.417
N6130  X-1.789 Y15.305
N6140  X-0.856 Y15.567
N6150  X0.064 Y15.811
N6160  X0.97 Y16.035
N6170  X1.861 Y16.241
N6180  X2.739 Y16.428
N6190  X3.603 Y16.596
N6200  X4.452 Y16.746
N6210  X5.288 Y16.876
N6220  X4.805 Y16.974
N6230  X4.304 Y17.079
N6240  X3.786 Y17.192
N6250  X3.251 Y17.312
N6260  X2.698 Y17.438
N6270  X2.127 Y17.572
N6280  X1.539 Y17.714
N6290  X0.934 Y17.862
N6300  X-11.925 Y20.971
N6310  X-11.931 Y23.632
N6320  X8.082 Y18.192
N6330  X8.087 Y15.641
N6340  X-7.142 Y11.391
N6350  X-7.375 Y11.326
N6360  X-7.596 Y11.264
N6370  X-8.004 Y11.151
N6380  X-8.191 Y11.099
N6390  X-8.367 Y11.051
N6400  X-8.53 Y11.006
N6410  X-8.823 Y10.927
N6420  X-8.953 Y10.892
N6430  X-9.071 Y10.862
N6440  X-9.177 Y10.834
N6450  X-9.272 Y10.81
N6460  X-9.355 Y10.789
N6470  X-9.427 Y10.772
N6480  X-9.487 Y10.758
N6490  X-9.149 Y10.681
N6500  X-8.823 Y10.606
N6510  X-8.511 Y10.531
N6520  X-8.211 Y10.458
N6530  X-7.924 Y10.386
N6540  X-7.649 Y10.315
N6550  X-7.388 Y10.245
N6560  X-7.139 Y10.177
N6570  X8.109 Y5.968
N6580  X8.115 Y3.267
N6590  X2.404 Y1.738
N6600  Z5
N6610  Z6
N6620  X3.333 Y24.666
N6630  Z5
N6640  Z0
N6650  X2.949 Y24.585
N6660  X2.551 Y24.518
N6670  X2.139 Y24.466
N6680  X1.714 Y24.428
N6690  X1.275 Y24.405
N6700  X0.823 Y24.397
N6710  X0.328 Y24.404
N6720  X-0.15 Y24.43
N6730  X-0.613 Y24.472
N6740  X-1.059 Y24.532
N6750  X-1.489 Y24.61
N6760  X-1.902 Y24.705
N6770  X-2.3 Y24.818
N6780  X-2.681 Y24.948
N6790  X-3.045 Y25.096
N6800  X-3.394 Y25.261
N6810  X-3.726 Y25.444
N6820  X-4.042 Y25.644
N6830  X-4.342 Y25.862
N6840  X-4.625 Y26.097
N6850  X-4.893 Y26.35
N6860  X-5.143 Y26.621
N6870  X-5.339 Y26.857
N6880  X-5.522 Y27.1
N6890  X-5.692 Y27.349
N6900  X-5.85 Y27.605
N6910  X-5.995 Y27.867
N6920  X-6.128 Y28.135
N6930  X-6.248 Y28.41
N6940  X-6.355 Y28.692
N6950  X-6.45 Y28.979
N6960  X-6.533 Y29.274
N6970  X-6.603 Y29.574
N6980  X-6.66 Y29.881
N6990  X-6.705 Y30.195
N7000  X-6.737 Y30.515
N7010  X-6.756 Y30.841
N7020  X-6.764 Y31.174
N7030  X-6.757 Y31.543
N7040  X-6.735 Y31.904
N7050  X-6.697 Y32.256
N7060  X-6.644 Y32.6
N7070  X-6.576 Y32.935
N7080  X-6.493 Y33.262
N7090  X-6.394 Y33.58
N7100  X-6.28 Y33.89
N7110  X-6.151 Y34.191
N7120  X-6.006 Y34.484
N7130  X-5.846 Y34.768
N7140  X-5.671 Y35.044
N7150  X-5.48 Y35.311
N7160  X-5.274 Y35.57
N7170  X-5.053 Y35.82
N7180  X-4.817 Y36.062
N7190  X-4.567 Y36.292
N7200  X-4.305 Y36.508
N7210  X-4.032 Y36.708
N7220  X-3.746 Y36.894
N7230  X-3.45 Y37.065
N7240  X-3.141 Y37.222
N7250  X-2.821 Y37.363
N7260  X-2.489 Y37.49
N7270  X-2.145 Y37.602
N7280  X-1.79 Y37.699
N7290  X-1.423 Y37.781
N7300  X-1.044 Y37.848
N7310  X-0.653 Y37.901
N7320  X-0.251 Y37.939
N7330  X0.163 Y37.962
N7340  X0.588 Y37.971
N7350  X0.933 Y37.968
N7360  X1.269 Y37.959
N7370  X1.595 Y37.943
N7380  X1.912 Y37.921
N7390  X2.22 Y37.892
N7400  X2.518 Y37.857
N7410  X2.807 Y37.815
N7420  X3.087 Y37.766
N7430  X3.357 Y37.711
N7440  X3.619 Y37.65
N7450  X3.87 Y37.581
N7460  X4.113 Y37.507
N7470  X4.346 Y37.425
N7480  X4.57 Y37.337
N7490  X4.785 Y37.243
N7500  X4.99 Y37.141
N7510  X5.38 Y36.921
N7520  X5.748 Y36.676
N7530  X6.093 Y36.407
N7540  X6.416 Y36.115
N7550  X6.717 Y35.798
N7560  X6.995 Y35.458
N7570  X7.251 Y35.093
N7580  X7.485 Y34.705
N7590  X7.694 Y34.3
N7600  X7.875 Y33.886
N7610  X8.028 Y33.463
N7620  X8.154 Y33.03
N7630  X8.252 Y32.589
N7640  X8.322 Y32.138
N7650  X8.365 Y31.677
N7660  X8.38 Y31.208
N7670  X8.373 Y30.832
N7680  X8.351 Y30.465
N7690  X8.314 Y30.108
N7700  X8.261 Y29.759
N7710  X8.193 Y29.42
N7720  X8.11 Y29.09
N7730  X8.012 Y28.768
N7740  X7.898 Y28.456
N7750  X7.769 Y28.153
N7760  X7.625 Y27.859
N7770  X7.466 Y27.574
N7780  X7.291 Y27.298
N7790  X7.101 Y27.031
N7800  X6.896 Y26.774
N7810  X6.676 Y26.525
N7820  X6.44 Y26.285
N7830  X6.19 Y26.058
N7840  X5.926 Y25.844
N7850  X5.649 Y25.646
N7860  X5.359 Y25.462
N7870  X5.055 Y25.293
N7880  X4.738 Y25.138
N7890  X4.407 Y24.998
N7900  X4.062 Y24.873
N7910  X3.705 Y24.762
N7920  X3.333 Y24.666
N7930  Z5
N7940  Z6
N7950  X4.983 Y28.144
N7960  Z5
N7970  Z0
N7980  X5.15 Y28.299
N7990  X5.307 Y28.458
N8000  X5.452 Y28.623
N8010  X5.587 Y28.792
N8020  X5.711 Y28.966
N8030  X5.824 Y29.145
N8040  X5.927 Y29.329
N8050  X6.018 Y29.518
N8060  X6.099 Y29.711
N8070  X6.169 Y29.91
N8080  X6.228 Y30.113
N8090  X6.276 Y30.321
N8100  X6.313 Y30.535
N8110  X6.34 Y30.753
N8120  X6.356 Y30.975
N8130  X6.361 Y31.203
N8140  X6.355 Y31.429
N8150  X6.338 Y31.651
N8160  X6.31 Y31.867
N8170  X6.272 Y32.079
N8180  X6.222 Y32.285
N8190  X6.162 Y32.487
N8200  X6.091 Y32.685
N8210  X6.009 Y32.877
N8220  X5.916 Y33.065
N8230  X5.812 Y33.247
N8240  X5.698 Y33.425
N8250  X5.572 Y33.598
N8260  X5.436 Y33.766
N8270  X5.289 Y33.93
N8280  X5.131 Y34.089
N8290  X4.962 Y34.242
N8300  X4.782 Y34.389
N8310  X4.591 Y34.526
N8320  X4.388 Y34.654
N8330  X4.174 Y34.772
N8340  X3.949 Y34.88
N8350  X3.712 Y34.979
N8360  X3.463 Y35.069
N8370  X3.204 Y35.149
N8380  X2.933 Y35.22
N8390  X2.65 Y35.281
N8400  X2.356 Y35.332
N8410  X2.051 Y35.374
N8420  X1.735 Y35.407
N8430  X1.407 Y35.43
N8440  X1.067 Y35.443
N8450  X0.717 Y35.447
N8460  X0.386 Y35.442
N8470  X0.065 Y35.426
N8480  X-0.245 Y35.402
N8490  X-0.546 Y35.368
N8500  X-0.836 Y35.324
N8510  X-1.115 Y35.271
N8520  X-1.385 Y35.209
N8530  X-1.644 Y35.137
N8540  X-1.894 Y35.055
N8550  X-2.133 Y34.964
N8560  X-2.362 Y34.863
N8570  X-2.58 Y34.753
N8580  X-2.789 Y34.633
N8590  X-2.987 Y34.504
N8600  X-3.175 Y34.365
N8610  X-3.353 Y34.217
N8620  X-3.52 Y34.062
N8630  X-3.677 Y33.902
N8640  X-3.822 Y33.737
N8650  X-3.957 Y33.568
N8660  X-4.081 Y33.394
N8670  X-4.194 Y33.216
N8680  X-4.297 Y33.033
N8690  X-4.388 Y32.845
N8700  X-4.469 Y32.653
N8710  X-4.539 Y32.456
N8720  X-4.598 Y32.255
N8730  X-4.646 Y32.049
N8740  X-4.684 Y31.838
N8750  X-4.71 Y31.623
N8760  X-4.726 Y31.403
N8770  X-4.731 Y31.178
N8780  X-4.725 Y30.951
N8790  X-4.708 Y30.728
N8800  X-4.681 Y30.51
N8810  X-4.643 Y30.297
N8820  X-4.594 Y30.089
N8830  X-4.534 Y29.886
N8840  X-4.464 Y29.688
N8850  X-4.383 Y29.494
N8860  X-4.291 Y29.306
N8870  X-4.188 Y29.122
N8880  X-4.074 Y28.944
N8890  X-3.95 Y28.771
N8900  X-3.815 Y28.602
N8910  X-3.67 Y28.438
N8920  X-3.513 Y28.279
N8930  X-3.346 Y28.125
N8940  X-3.168 Y27.979
N8950  X-2.979 Y27.842
N8960  X-2.779 Y27.714
N8970  X-2.568 Y27.596
N8980  X-2.346 Y27.487
N8990  X-2.113 Y27.388
N9000  X-1.87 Y27.299
N9010  X-1.615 Y27.219
N9020  X-1.349 Y27.148
N9030  X-1.072 Y27.087
N9040  X-0.785 Y27.036
N9050  X-0.486 Y26.994
N9060  X-0.177 Y26.961
N9070  X0.144 Y26.938
N9080  X0.475 Y26.925
N9090  X0.818 Y26.921
N9100  X1.16 Y26.926
N9110  X1.492 Y26.941
N9120  X1.812 Y26.966
N9130  X2.122 Y27
N9140  X2.42 Y27.043
N9150  X2.708 Y27.096
N9160  X2.985 Y27.158
N9170  X3.25 Y27.23
N9180  X3.505 Y27.311
N9190  X3.749 Y27.402
N9200  X3.982 Y27.502
N9210  X4.204 Y27.611
N9220  X4.415 Y27.73
N9230  X4.615 Y27.859
N9240  X4.805 Y27.997
N9250  X4.983 Y28.144
N9260  Z5
N9270  Z6
N9280  X5.133 Y40.832
N9290  Z5
N9300  Z0
N9310  X-6.458 Y40.806
N9320  X-6.463 Y43.016
N9330  X-4.266 Y43.021
N9340  X-4.636 Y43.23
N9350  X-4.972 Y43.434
N9360  X-5.276 Y43.635
N9370  X-5.547 Y43.832
N9380  X-5.785 Y44.024
N9390  X-5.99 Y44.213
N9400  X-6.163 Y44.398
N9410  X-6.302 Y44.579
N9420  X-6.418 Y44.76
N9430  X-6.518 Y44.945
N9440  X-6.603 Y45.135
N9450  X-6.673 Y45.33
N9460  X-6.727 Y45.529
N9470  X-6.766 Y45.732
N9480  X-6.789 Y45.941
N9490  X-6.797 Y46.153
N9500  X-6.785 Y46.464
N9510  X-6.749 Y46.777
N9520  X-6.688 Y47.09
N9530  X-6.602 Y47.405
N9540  X-6.492 Y47.722
N9550  X-6.356 Y48.04
N9560  X-6.196 Y48.359
N9570  X-6.012 Y48.679
N9580  X-3.731 Y47.838
N9590  X-3.856 Y47.613
N9600  X-3.963 Y47.388
N9610  X-4.054 Y47.162
N9620  X-4.128 Y46.937
N9630  X-4.186 Y46.712
N9640  X-4.227 Y46.486
N9650  X-4.252 Y46.261
N9660  X-4.259 Y46.036
N9670  X-4.251 Y45.838
N9680  X-4.228 Y45.644
N9690  X-4.19 Y45.456
N9700  X-4.137 Y45.272
N9710  X-4.068 Y45.094
N9720  X-3.984 Y44.922
N9730  X-3.886 Y44.754
N9740  X-3.772 Y44.591
N9750  X-3.645 Y44.437
N9760  X-3.506 Y44.294
N9770  X-3.355 Y44.163
N9780  X-3.192 Y44.043
N9790  X-3.018 Y43.935
N9800  X-2.833 Y43.839
N9810  X-2.635 Y43.754
N9820  X-2.426 Y43.68
N9830  X-2.094 Y43.585
N9840  X-1.755 Y43.503
N9850  X-1.409 Y43.433
N9860  X-1.054 Y43.376
N9870  X-0.692 Y43.332
N9880  X-0.322 Y43.301
N9890  X0.055 Y43.283
N9900  X0.44 Y43.277
N9910  X8.025 Y43.294
N9920  X8.031 Y40.839
N9930  X5.133 Y40.832
N9940  Z5
N9950  Z6
N9960  X8.01 Y50.116
N9970  Z5
N9980  Z0
N9990  X-11.99 Y50.071
N10000  X-11.996 Y52.526
N10010  X8.004 Y52.571
N10020  X8.01 Y50.116
N10030  Z5
N10040  Z6
N10050  X4.701 Y56.281
N10060  Z5
N10070  Z0
N10080  X4.266 Y56.093
N10090  X3.814 Y55.93
N10100  X3.346 Y55.792
N10110  X2.863 Y55.679
N10120  X2.363 Y55.59
N10130  X1.847 Y55.527
N10140  X1.315 Y55.488
N10150  X0.767 Y55.474
N10160  X0.231 Y55.485
N10170  X-0.292 Y55.517
N10180  X-0.803 Y55.573
N10190  X-1.302 Y55.651
N10200  X-1.788 Y55.751
N10210  X-2.261 Y55.874
N10220  X-2.722 Y56.02
N10230  X-3.17 Y56.189
N10240  X-3.599 Y56.38
N10250  X-4.003 Y56.594
N10260  X-4.381 Y56.83
N10270  X-4.733 Y57.089
N10280  X-5.059 Y57.371
N10290  X-5.359 Y57.675
N10300  X-5.634 Y58.002
N10310  X-5.883 Y58.352
N10320  X-6.105 Y58.718
N10330  X-6.297 Y59.095
N10340  X-6.459 Y59.483
N10350  X-6.593 Y59.882
N10360  X-6.696 Y60.291
N10370  X-6.771 Y60.711
N10380  X-6.816 Y61.141
N10390  X-6.832 Y61.583
N10400  X-6.824 Y61.906
N10410  X-6.799 Y62.22
N10420  X-6.756 Y62.525
N10430  X-6.696 Y62.821
N10440  X-6.619 Y63.109
N10450  X-6.525 Y63.387
N10460  X-6.413 Y63.657
N10470  X-6.284 Y63.917
N10480  X-6.142 Y64.167
N10490  X-5.988 Y64.406
N10500  X-5.825 Y64.632
N10510  X-5.65 Y64.846
N10520  X-5.466 Y65.049
N10530  X-5.271 Y65.239
N10540  X-5.065 Y65.418
N10550  X-4.849 Y65.585
N10560  X-12.025 Y65.569
N10570  X-12.03 Y68.011
N10580  X7.97 Y68.055
N10590  X7.975 Y65.777
N10600  X6.147 Y65.773
N10610  X6.408 Y65.596
N10620  X6.653 Y65.41
N10630  X6.88 Y65.213
N10640  X7.091 Y65.005
N10650  X7.285 Y64.788
N10660  X7.463 Y64.56
N10670  X7.623 Y64.323
N10680  X7.767 Y64.075
N10690  X7.894 Y63.817
N10700  X8.004 Y63.549
N10710  X8.097 Y63.27
N10720  X8.174 Y62.982
N10730  X8.233 Y62.683
N10740  X8.276 Y62.374
N10750  X8.302 Y62.055
N10760  X8.311 Y61.726
N10770  X8.297 Y61.297
N10780  X8.253 Y60.877
N10790  X8.18 Y60.466
N10800  X8.076 Y60.063
N10810  X7.943 Y59.669
N10820  X7.78 Y59.283
N10830  X7.586 Y58.907
N10840  X7.363 Y58.538
N10850  X7.114 Y58.185
N10860  X6.84 Y57.852
N10870  X6.543 Y57.539
N10880  X6.222 Y57.247
N10890  X5.877 Y56.975
N10900  X5.509 Y56.723
N10910  X5.117 Y56.492
N10920  X4.701 Y56.281
N10930  Z5
N10940  Z6
N10950  X5.645 Y59.957
N10960  Z5
N10970  Z0
N10980  X5.757 Y60.122
N10990  X5.859 Y60.291
N11000  X5.95 Y60.462
N11010  X6.031 Y60.637
N11020  X6.1 Y60.815
N11030  X6.159 Y60.997
N11040  X6.207 Y61.182
N11050  X6.244 Y61.37
N11060  X6.271 Y61.561
N11070  X6.287 Y61.756
N11080  X6.292 Y61.953
N11090  X6.286 Y62.153
N11100  X6.27 Y62.348
N11110  X6.244 Y62.54
N11120  X6.208 Y62.728
N11130  X6.161 Y62.912
N11140  X6.104 Y63.092
N11150  X6.037 Y63.269
N11160  X5.959 Y63.442
N11170  X5.871 Y63.61
N11180  X5.773 Y63.775
N11190  X5.665 Y63.937
N11200  X5.546 Y64.094
N11210  X5.417 Y64.248
N11220  X5.278 Y64.398
N11230  X5.129 Y64.544
N11240  X4.969 Y64.686
N11250  X4.799 Y64.822
N11260  X4.618 Y64.949
N11270  X4.426 Y65.067
N11280  X4.223 Y65.177
N11290  X4.01 Y65.277
N11300  X3.785 Y65.369
N11310  X3.55 Y65.452
N11320  X3.304 Y65.526
N11330  X3.048 Y65.592
N11340  X2.78 Y65.648
N11350  X2.502 Y65.696
N11360  X2.213 Y65.735
N11370  X1.913 Y65.765
N11380  X1.602 Y65.786
N11390  X1.281 Y65.799
N11400  X0.949 Y65.802
N11410  X0.583 Y65.797
N11420  X0.23 Y65.783
N11430  X-0.111 Y65.76
N11440  X-0.439 Y65.728
N11450  X-0.755 Y65.687
N11460  X-1.059 Y65.637
N11470  X-1.35 Y65.578
N11480  X-1.629 Y65.51
N11490  X-1.896 Y65.433
N11500  X-2.15 Y65.348
N11510  X-2.391 Y65.253
N11520  X-2.621 Y65.15
N11530  X-2.838 Y65.037
N11540  X-3.042 Y64.916
N11550  X-3.235 Y64.786
N11560  X-3.414 Y64.647
N11570  X-3.583 Y64.501
N11580  X-3.74 Y64.351
N11590  X-3.886 Y64.197
N11600  X-4.022 Y64.039
N11610  X-4.146 Y63.877
N11620  X-4.26 Y63.71
N11630  X-4.363 Y63.54
N11640  X-4.455 Y63.365
N11650  X-4.536 Y63.187
N11660  X-4.607 Y63.004
N11670  X-4.666 Y62.817
N11680  X-4.714 Y62.625
N11690  X-4.752 Y62.43
N11700  X-4.779 Y62.231
N11710  X-4.795 Y62.027
N11720  X-4.8 Y61.819
N11730  X-4.794 Y61.617
N11740  X-4.778 Y61.419
N11750  X-4.751 Y61.225
N11760  X-4.714 Y61.035
N11770  X-4.667 Y60.849
N11780  X-4.609 Y60.668
N11790  X-4.541 Y60.491
N11800  X-4.462 Y60.318
N11810  X-4.373 Y60.149
N11820  X-4.273 Y59.985
N11830  X-4.163 Y59.825
N11840  X-4.043 Y59.669
N11850  X-3.912 Y59.517
N11860  X-3.77 Y59.369
N11870  X-3.619 Y59.226
N11880  X-3.456 Y59.087
N11890  X-3.283 Y58.954
N11900  X-3.098 Y58.831
N11910  X-2.9 Y58.715
N11920  X-2.691 Y58.608
N11930  X-2.469 Y58.51
N11940  X-2.235 Y58.421
N11950  X-1.99 Y58.34
N11960  X-1.732 Y58.267
N11970  X-1.463 Y58.204
N11980  X-1.181 Y58.148
N11990  X-0.887 Y58.102
N12000  X-0.582 Y58.064
N12010  X-0.264 Y58.035
N12020  X0.066 Y58.014
N12030  X0.408 Y58.002
N12040  X0.762 Y57.998
N12050  X1.104 Y58.004
N12060  X1.435 Y58.018
N12070  X1.756 Y58.042
N12080  X2.065 Y58.075
N12090  X2.364 Y58.117
N12100  X2.651 Y58.168
N12110  X2.927 Y58.228
N12120  X3.193 Y58.297
N12130  X3.447 Y58.376
N12140  X3.69 Y58.463
N12150  X3.923 Y58.56
N12160  X4.144 Y58.666
N12170  X4.355 Y58.781
N12180  X4.554 Y58.905
N12190  X4.742 Y59.039
N12200  X4.92 Y59.181
N12210  X5.086 Y59.33
N12220  X5.242 Y59.482
N12230  X5.387 Y59.637
N12240  X5.521 Y59.795
N12250  X5.645 Y59.957
N12260  Z5
N12270  Z6
N12280  X5.163 Y72.606
N12290  Z5
N12300  Z0
N12310  X5.156 Y75.403
N12320  X7.953 Y75.409
N12330  X7.959 Y72.612
N12340  X5.163 Y72.606
N12350  Z5
N12360  Z10
N12370 M400 (clear output 0)
