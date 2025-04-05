/**
  ******************************************************************************
  * @file    ann_data_params.c
  * @author  AST Embedded Analytics Research Platform
  * @date    2025-04-05T14:25:42+0800
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#include "ann_data_params.h"


/**  Activations Section  ****************************************************/
ai_handle g_ann_activations_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(NULL),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};




/**  Weights Section  ********************************************************/
AI_ALIGNED(32)
const ai_u64 s_ann_weights_array_u64[263] = {
  0xbee2682cbf11feebU, 0xbf00f8d43f2c6b9aU, 0xbefefb8cbd6f55b0U, 0x3d4a5130be996d17U,
  0xbebb88a53ba69100U, 0x3f03678b3eaec839U, 0xbe2087acbee9416dU, 0x3ec9eac0be8e7670U,
  0x3ee36a63bf00773bU, 0x3f0bb05ebd18d4b7U, 0xbec54d96bf21bd9fU, 0xbd6e28e0beab35a1U,
  0x3effb5f53e62f7e8U, 0x3d4ba740bf0cd722U, 0xbe220fccbf164e15U, 0x0U,
  0x3e8cfe8600000000U, 0x3d83d99f00000000U, 0x3d8545e5U, 0xbaa93ed2U,
  0x3e5d8660bed677fcU, 0xbba212d5bd3017c0U, 0x3de46c0f3edd3a66U, 0x3e91deaabe9e092fU,
  0xbd133f60be88ac69U, 0xbe14398a3ed87f00U, 0x3f047506bee00a0dU, 0xbf0690c93ecbb13eU,
  0x3f0b799a3e3ba2d9U, 0x3e124cccbdbd8b95U, 0x3e0799ac3da725f0U, 0x3ec462463ec8bfe6U,
  0xbe85c64a3e94605aU, 0xbec854e83e33a4f4U, 0xbcdfa8c0bd96bc18U, 0xbdb62e4c3de15aa8U,
  0x3dde046bbec9d746U, 0x3e261c113ec16006U, 0xbe5af3763e1b736aU, 0xbe96fceb3ec89e37U,
  0x3e2f7e5cbee9d472U, 0x3d89a2733ef2cd6cU, 0x3d949224be9571e9U, 0xbe80602dbe0f64f8U,
  0xbd989218bebb3c0cU, 0x3effa6e43efa177cU, 0xbe52f61fbe7d566eU, 0xbeb0eea8be947618U,
  0x3eea2f44be9082dbU, 0xbdb080503ca0553dU, 0x3ea218a4bec75ecfU, 0x3e8cc9aebe076c60U,
  0x3e817668bf031160U, 0xbd3a0a003ec13cfcU, 0xbdb09784be4ca69bU, 0x3e5b04103f0367aaU,
  0xbc65b8573e96ecccU, 0x3e2e58993df82730U, 0x3dafbc70bf10d0b1U, 0x3eea61543f0c6521U,
  0x3ee035b23d126800U, 0x3cbd5264be3c6fbeU, 0xbdf079ca3eba271aU, 0x3dfbebf03eca3243U,
  0x3e90d9063e97e15aU, 0xbe4eab323d85d3b8U, 0xbe108c593f0a93c0U, 0xbeb7ec763edb2f94U,
  0x3ec2737abefb59e2U, 0x3e4ce6b43e998fa8U, 0x41677cf4bf176f87U, 0xbddc4ab34108fcbcU,
  0xbfe1e2ffbd702193U, 0xc055b0083e80bdc2U, 0xbfefbebc3e9ef505U, 0x3e934e78bef3ba54U,
  0x3ec688563f358041U, 0x400662aebe212c1cU, 0xc088f708c1adc502U, 0xbef668e73f536c45U,
  0xbe7cdae441cfbc82U, 0xc020e25e3f0494d1U, 0xbff6adfc4089b90cU, 0x3f864741bf0d2040U,
  0xc0032612c11abc4dU, 0x3f06ad1fbf9c00b0U, 0x4004cc463f1061ecU, 0xbf3001dac02d5430U,
  0xc03e0625c005382fU, 0x412eac993fb833a1U, 0x401d78d43b411077U, 0x3dd847773e0ada86U,
  0x40b32149bf490e24U, 0xc02e9adc4181c099U, 0x41056f3a3f201311U, 0xbe89fe2541149b98U,
  0xbfaca493bff59e52U, 0x3f47cef33fa0dd7aU, 0x3fddd84b40d6cc0dU, 0xc09399b9c0db1ac1U,
  0xbf2a279abeb15d54U, 0x405d6af2400cd1fbU, 0x40dbf5f3be8cac30U, 0x403c8f2bbf933d17U,
  0x41851c6b3eb1493aU, 0xbf9a345fbe18e956U, 0xbe811cfac0349debU, 0x3fc542eabf141c99U,
  0x40aa24a2c0a12ac5U, 0x40f9e937bf527777U, 0x3f1477063f088d2bU, 0xc092caebbf8aaa4dU,
  0x407cfea33eb33c02U, 0xc0a5c9bec0072e6eU, 0x40bf28b7be7a2212U, 0x40f4b193bf040401U,
  0x4022fa7a3fe1411bU, 0x3f2b338a3d27df57U, 0xc02843d740def6bbU, 0x3f06b3a33ea9536cU,
  0x404e94cbc023b1d4U, 0x3fae37383f81211dU, 0xbf0b2f3fbf6b3b9dU, 0x40033306401f99a8U,
  0xc00ce78b4099ba08U, 0x3b7bc4bcbf9c1413U, 0x3e910a5b3dd6c176U, 0xbe5c0104bf08a631U,
  0x3e82b9bcbee829c8U, 0xbe0c73de3cef4345U, 0xbe9e102c3e95ab0fU, 0xbf0e57cb3f52e369U,
  0x3e530894bc3be322U, 0x3b3b5eef3e2cd346U, 0xbd3e9e573f112627U, 0x3f10d223bc82183bU,
  0xbdf18e503e13ef7eU, 0x3e06b914bda04565U, 0x3eef56e53e9109f9U, 0x3f03850c3f3f3e36U,
  0xbc3dabb9bd37bda0U, 0x3c2d5c433fe54456U, 0x3e9eb6c8bd77cd33U, 0x3caaf82fbd08ec2cU,
  0x3dca529f3ca03365U, 0x3f0bf642bd55941cU, 0x3dfd6b56bcebbc5dU, 0xbc551b173e74f065U,
  0xbeaa48ba3eb1d21dU, 0x3e95a8803f15b51aU, 0xbf78cad8bdfc389eU, 0xbe82c2d23ea0c91dU,
  0xbd94fd0c3ebe1f6eU, 0xbe19b27fbe6461f8U, 0x3eb09987be520324U, 0xbdb72a6cbe8e05bbU,
  0x3c9b92063f13e5dfU, 0x3eea45a3be9bb47fU, 0x3e5dc702be3b611bU, 0xbdd468e23e807c2dU,
  0x3f31440dbf885e63U, 0x3d4e779b3c21b99eU, 0x3e3519bbbe8f1f49U, 0x3ecd8ef13e2fc112U,
  0xbdb12cbabde4e382U, 0xbf5a6e95bda72472U, 0xbf42558bbdd31955U, 0x3e07dbdebe16dabaU,
  0xbf24c5213e23b39cU, 0xbe7896a93da4f865U, 0xbf8c05173dcad30bU, 0x3e1447f7bde022a8U,
  0x3e4a84593f184a8eU, 0xbc93d1e0be8048b9U, 0xbe894a79be294b3bU, 0xbe73e163bd3b7a2aU,
  0xbe2390a3be870aa1U, 0xbe5e41a43d6fc03bU, 0xbd7ab111be2d0211U, 0x3e1bc96abe4a2c49U,
  0x3e301a51bebe8e6eU, 0x3dc59585beba0ea7U, 0xbf20069bbebc9467U, 0x3e0f6ec7bd219eabU,
  0xbd3365973eb78f68U, 0xbdc7cc7c3d3b4bc5U, 0xbe17b9323d3bf407U, 0x3e81d690be686c88U,
  0x3ca195e6be3a7578U, 0x3e292d7b3e72029eU, 0x3e109155bbc0f813U, 0x3eef2b6ebde4d495U,
  0x3db5ab95bcf221b0U, 0xbd304a943e20963aU, 0x3e901d933d03bae2U, 0x3dcd2c55bb05c7baU,
  0xbdac324f3de6e19bU, 0xbe08de743e307635U, 0xbcb6d4a43d44d175U, 0xbe72fb453df62beaU,
  0xbb73b8773f0db3f6U, 0xbcb027cebb27fe94U, 0xbdfd4c073e48f229U, 0x3ec127783e6f6e7bU,
  0xbea274d03e92db94U, 0x3d48206d3c958ab5U, 0xbbbb338abe558b0aU, 0x3ee6af14be60f2ddU,
  0xbdde9a0c3d4ec6eeU, 0x3f220c2a3e44c3bdU, 0xbdb39ee93f1ec64aU, 0xbccc8fa7bdb1bcf7U,
  0xbe25133cbe19729eU, 0x3f0c52fd3e8f9878U, 0xbec1b0653e5e9c3fU, 0xbe0fa7c4bc996045U,
  0x3e3330d8bda27838U, 0x3d796f523c37d8fcU, 0xbcd4d16fbe24c893U, 0x3e81e7e3bdc97770U,
  0xbdaea00b3f1b60d9U, 0xbcd000a6bdb3be0bU, 0xbe2271ffbe19405bU, 0x3f0a27b23e8bbfbbU,
  0xbebbb57b3e566898U, 0x3e948b793f45ac68U, 0xbdfa49ed3f340c33U, 0xbdb70621be97616eU,
  0x3edbd8083f9a14d0U, 0x3f6287973e282891U, 0xbb17f7513b665e75U, 0x3b78eef0bb3227bfU,
  0xbe78c6ebba462e63U, 0xbe5f5b4a3ab779acU, 0x3ede898abc176820U, 0x3bb6c4253e88f118U,
  0xbe3cc2813e5c697cU, 0x3e39d064be7da9d7U, 0xbe0e82443d142bc2U, 0xbcd13539be0a7686U,
  0x3e7ab9a5bdd045caU, 0xbd30dc673dcb483fU, 0x3dc5a29fbe625c72U, 0xbd904bcf3ec41ab0U,
  0xbd1d5a0abc2db704U, 0x39961d61ba310328U, 0x3a6dfd5dba934f9bU, 0x3b45fb5eb983e6eaU,
  0xbef3ce9b3a20c256U, 0x3ef7247438f2c69fU, 0xbb9a9ae6bc25c8aeU, 0xbecb0e513c1a210cU,
  0xbb1c627bbcb3205aU, 0x3b404342bc6a4ab6U, 0x3c4f51453b0f505bU, 0xbc4f3aef3d14bc98U,
  0xbc514fecbc325cacU, 0xbb08d932bd06dc63U, 0xbb69a462bca3f67eU, 0x3c00a00ebcb8b319U,
  0x3e8997c8bafc33b1U, 0x39520bd53ea13394U, 0x3f58b2ff3f0757b9U,
};


ai_handle g_ann_weights_table[1 + 2] = {
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
  AI_HANDLE_PTR(s_ann_weights_array_u64),
  AI_HANDLE_PTR(AI_MAGIC_MARKER),
};

