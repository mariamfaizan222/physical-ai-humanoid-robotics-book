import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/physical-ai-book/__docusaurus/debug',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug', '12f'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/config',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/config', '4d3'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/content',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/content', 'a5b'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/globalData',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/globalData', 'abe'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/metadata',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/metadata', '587'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/registry',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/registry', '2ef'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/routes',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/routes', '1a5'),
    exact: true
  },
  {
    path: '/physical-ai-book/markdown-page',
    component: ComponentCreator('/physical-ai-book/markdown-page', '0d8'),
    exact: true
  },
  {
    path: '/physical-ai-book/docs',
    component: ComponentCreator('/physical-ai-book/docs', '160'),
    routes: [
      {
        path: '/physical-ai-book/docs',
        component: ComponentCreator('/physical-ai-book/docs', 'f84'),
        routes: [
          {
            path: '/physical-ai-book/docs',
            component: ComponentCreator('/physical-ai-book/docs', '8ac'),
            routes: [
              {
                path: '/physical-ai-book/docs/intro',
                component: ComponentCreator('/physical-ai-book/docs/intro', 'f2d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/Introduction',
                component: ComponentCreator('/physical-ai-book/docs/Introduction', 'e02'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/module-1/chapter-1',
                component: ComponentCreator('/physical-ai-book/docs/module-1/chapter-1', '87d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module-1/chapter-2',
                component: ComponentCreator('/physical-ai-book/docs/module-1/chapter-2', 'b36'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module-1/chapter-3',
                component: ComponentCreator('/physical-ai-book/docs/module-1/chapter-3', '30d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module-1/chapter-4',
                component: ComponentCreator('/physical-ai-book/docs/module-1/chapter-4', 'aed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module-1/chapter-5',
                component: ComponentCreator('/physical-ai-book/docs/module-1/chapter-5', '167'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module-2/chapter-6',
                component: ComponentCreator('/physical-ai-book/docs/module-2/chapter-6', '17f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module-2/chapter-7',
                component: ComponentCreator('/physical-ai-book/docs/module-2/chapter-7', '97b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module-2/chapter-8',
                component: ComponentCreator('/physical-ai-book/docs/module-2/chapter-8', 'ca0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module-3/chapter-10',
                component: ComponentCreator('/physical-ai-book/docs/module-3/chapter-10', '2ca'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module-3/chapter-11',
                component: ComponentCreator('/physical-ai-book/docs/module-3/chapter-11', '0e1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module-3/chapter-9',
                component: ComponentCreator('/physical-ai-book/docs/module-3/chapter-9', '109'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module-4/chapter-13',
                component: ComponentCreator('/physical-ai-book/docs/module-4/chapter-13', 'ef6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/module-4/chapter-14',
                component: ComponentCreator('/physical-ai-book/docs/module-4/chapter-14', '56d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/docs/Module1_IntroPhysicalAI',
                component: ComponentCreator('/physical-ai-book/docs/Module1_IntroPhysicalAI', 'abb'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/Module1_ROSArchitecture',
                component: ComponentCreator('/physical-ai-book/docs/Module1_ROSArchitecture', 'eb9'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/Module1_ROSComm',
                component: ComponentCreator('/physical-ai-book/docs/Module1_ROSComm', 'c10'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/Module1_ROSPackages',
                component: ComponentCreator('/physical-ai-book/docs/Module1_ROSPackages', '249'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/Module1_SensorSystems',
                component: ComponentCreator('/physical-ai-book/docs/Module1_SensorSystems', 'a3f'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/Module2_DigitalTwin',
                component: ComponentCreator('/physical-ai-book/docs/Module2_DigitalTwin', 'bc2'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/Module3_NVIDIAIsaac',
                component: ComponentCreator('/physical-ai-book/docs/Module3_NVIDIAIsaac', '366'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/Module4_ActionGeneration',
                component: ComponentCreator('/physical-ai-book/docs/Module4_ActionGeneration', '4e8'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/Module4_Challenges',
                component: ComponentCreator('/physical-ai-book/docs/Module4_Challenges', '777'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/Module4_FutureDirections',
                component: ComponentCreator('/physical-ai-book/docs/Module4_FutureDirections', 'fec'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/Module4_LanguageUnderstanding',
                component: ComponentCreator('/physical-ai-book/docs/Module4_LanguageUnderstanding', 'f3e'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/Module4_MultimodalFusion',
                component: ComponentCreator('/physical-ai-book/docs/Module4_MultimodalFusion', '11d'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/Module4_TaskNavLLM',
                component: ComponentCreator('/physical-ai-book/docs/Module4_TaskNavLLM', 'ec4'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/Module4_UseCases',
                component: ComponentCreator('/physical-ai-book/docs/Module4_UseCases', 'a2a'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/Module4_VisualPerception',
                component: ComponentCreator('/physical-ai-book/docs/Module4_VisualPerception', '966'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/Module4_VLA_Intro',
                component: ComponentCreator('/physical-ai-book/docs/Module4_VLA_Intro', '2f1'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/Module4_VLAPipelines',
                component: ComponentCreator('/physical-ai-book/docs/Module4_VLAPipelines', '631'),
                exact: true
              },
              {
                path: '/physical-ai-book/docs/Module4_VoiceControl',
                component: ComponentCreator('/physical-ai-book/docs/Module4_VoiceControl', '8aa'),
                exact: true
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/physical-ai-book/',
    component: ComponentCreator('/physical-ai-book/', 'dbb'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
