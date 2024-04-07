### 函数效率测试

| isTwoSegmentsCollinearCoincident | optimal                           | time                  |
| -------------------------------- | --------------------------------- | --------------------- |
| call 173480                      | <br />cross norm                  | 10ms                  |
| normalized更加耗时               | normalized<br />cross squaredNorm | 11ms                  |
|                                  | normalized<br />_getDistance      | 12ms                  |
|                                  | squaredNorm<br />cross norm       | 5.510 5.386 5.452     |
|                                  | norm<br />cross norm              | 5.350 5.205 5.201 ms. |



随机测试



关omp，box缩小，

| create time:         | Fri_Feb__2_10_52_54 |
| -------------------- | ------------------- |
| countP_graphic       | 665                 |
| countP_mesh          | 11101               |
| countP_triangle      | 271820              |
| countP_vertex        | 156849              |
| countP_trigon        | 72015               |
| count_inside_judge   | 46845               |
| count_tri_inter_pre  | 18085718            |
| count_tri_inter_ed   | 1928213             |
| count_front_judge    | 36079               |
| count_bool_diff      | 1798406             |
| count_bool_union     | 1802                |
| count_hidden_jump    | 1575373             |
| count_union_empty    | 211                 |
| count_tri_degen      | 0                   |
| count_trigon_empty   | 0                   |
| count_trigon_hidden  | 59281               |
| count_tole_angle     | 1598                |
| count_trigon_area0   | 775                 |
| count_contour_area0  | 0                   |
| time_mesh            | 0.097000s           |
| time_assembly_trigon | 0.022000s           |
| time_kdtree          | 0.179000s           |
| time_frontJudge      | 0.534000s           |
| time_difference      | 1.982000s           |
| time_union           | 0.496000s           |
| time_all             | 3.225000s           |

| create time:         | Fri_Feb_23_17_47_55 |
| -------------------- | ------------------- |
| countP_graphic       | 665                 |
| countP_mesh          | 11101               |
| countP_triangle      | 271820              |
| countP_vertex        | 156849              |
| countP_trigon        | 65501               |
| countP_contour       | 11063               |
| count_tri_invisible  | 206159              |
| count_tri_degen      | 160                 |
| count_tri_inter_pre  | 15522720            |
| count_hidden_jump    | 7515                |
| count_boxA_front     | 7457078             |
| count_tri_inter_sat  | 8019213             |
| count_tri_sat_sepa   | 4962659             |
| count_front_judge    | 3056554             |
| count_tri_coplanar   | 840                 |
| count_front_unknown  | 1736475             |
| count_triB_front     | 1044                |
| count_trigon_inside  | 496                 |
| count_bool_diff      | 526                 |
| count_bool_union     | 64942               |
| count_union_empty    | 0                   |
| count_trigon_hidden  | 559                 |
| count_trigon_area0   | 0                   |
| count_contour_area0  | 0                   |
| time_mesh_convert    | 0.078000s           |
| time_assembly_trigon | 0.021000s           |
| time_kdtree          | 0.155000s           |
| time_frontJudge      | 1.759000s           |
| time_difference      | 0.010000s           |
| time_union           | 0.462000s           |
| time_all             | 2.467000s           |



| create time:         | Sat_Feb_24_08_21_11 |
| -------------------- | ------------------- |
| countP_graphic       | 665                 |
| countP_mesh          | 11101               |
| countP_triangle      | 271820              |
| countP_vertex        | 156849              |
| countP_trigon        | 65501               |
| countP_contour       | 5094                |
| count_tri_invisible  | 206159              |
| count_tri_degen      | 160                 |
| count_tri_inter_pre  | 15522720            |
| count_hidden_jump    | 1002865             |
| count_boxA_front     | 3994999             |
| count_tri_inter_sat  | 4356066             |
| count_tri_sat_sepa   | 2907076             |
| count_front_judge    | 1448990             |
| count_tri_coplanar   | 623                 |
| count_front_unknown  | 2951                |
| count_triB_front     | 1440500             |
| count_trigon_inside  | 28238               |
| count_bool_diff      | 1339523             |
| count_bool_union     | 31232               |
| count_union_empty    | 0                   |
| count_trigon_hidden  | 34269               |
| count_trigon_area0   | 0                   |
| count_contour_area0  | 0                   |
| time_mesh_convert    | 0.096000s           |
| time_assembly_trigon | 0.021000s           |
| time_kdtree          | 0.154000s           |
| time_frontJudge      | 0.960000s           |
| time_difference      | 6.177000s           |
| time_union           | 0.393000s           |
| time_all             | 7.742000s           |

| create time:           | Sat_Feb_24_23_02_24 |
| ---------------------- | ------------------- |
| countP_graphic         | 689                 |
| countP_mesh            | 11101               |
| countP_triangle        | 271820              |
| countP_vertex          | 156849              |
| countP_trigon          | 54509               |
| countP_contour         | 1275                |
| count_tri_invisible    | 206159              |
| count_tri_degene_area  | 487                 |
| count_tri_degene_angle | 10665               |
| count_tri_inter_pre    | 12738360            |
| count_hidden_jump      | 880735              |
| count_boxA_front       | 2008679             |
| count_tri_inter_sat    | 1646147             |
| count_tri_sat_sepa     | 322386              |
| count_front_judge      | 1323761             |
| count_tri_coplanar     | 552                 |
| count_front_unknown    | 0                   |
| count_triB_front       | 1319440             |
| count_trigon_inside    | 37848               |
| count_bool_diff        | 1193414             |
| count_bool_union       | 7243                |
| count_union_empty      | 0                   |
| count_trigon_hidden    | 47266               |
| count_trigon_area0     | 0                   |
| count_contour_area0    | 0                   |
| time_mesh_convert      | 0.110000s           |
| time_assembly_trigon   | 0.018000s           |
| time_kdtree            | 0.133000s           |
| time_frontJudge        | 0.724000s           |
| time_difference        | 1.271000s           |
| time_union             | 0.287000s           |
| time_all               | 2.440000s           |

test



| create time:           | Tue_Mar_26_17_23_05 |
| ---------------------- | ------------------- |
| countP_graphic         | 689                 |
| countP_mesh            | 11101               |
| countP_triangle        | 271820              |
| countP_vertex          | 156849              |
| countP_trigon          | 65302               |
| countP_contour         | 1244                |
| count_tri_invisible    | 204361              |
| count_tri_degene_area  | 831                 |
| count_tri_degene_angle | 0                   |
| count_tri_degene_dist  | 1326                |
| count_tri_inter_pre    | 15876560            |
| count_hidden_jump      | 1301414             |
| count_boxA_front       | 2898387             |
| count_tri_inter_sat    | 2333809             |
| count_tri_sat_sepa     | 493835              |
| count_front_judge      | 1839974             |
| count_tri_coplanar     | 1615                |
| count_triB_front       | 1820396             |
| count_trigon_inside    | 44845               |
| count_front_unknown    | 0                   |
| count_bidirect_shield  | 0                   |
| count_front_ambiguous  | 0                   |
| count_tri_inter_3d     | 0                   |
| count_bool_diff        | 1599725             |
| count_bool_union       | 8369                |
| count_union_empty      | 0                   |
| count_trigon_hidden    | 56933               |
| count_trigon_area0     | 0                   |
| count_contour_area0    | 0                   |
| time_mesh_convert      | 0.082000s           |
| time_assembly_trigon   | 0.020000s           |
| time_bvhtree           | 0.032000s           |
| time_frontJudge        | 1.292000s           |
| time_difference        | 2.221000s           |
| time_union             | 0.453000s           |
| time_all               | 4.020000s           |



