import { RouteRecordRaw } from 'vue-router';

const routes: RouteRecordRaw[] = [
  {
    path: '/',
    component: () => import('@/views/IndexView.vue'),
    redirect: '/welcome',
    children: [
      {
        path: 'login',
        component: () => import('@/views/LoginView.vue'),
      },
      {
        path: 'mapping',
        component: () => import('@/views/MappingView.vue'),
      },
      {
        path: 'control',
        component: () => import('@/views/RoboControl.vue'),
      },
      {
        path: 'settings',
        component: () => import('@/views/SettingsView.vue'),
      },
      {
        path: 'welcome',
        component: () => import('@/views/WelcomeView.vue'),
      },
    ],
  },

  // Always leave this as last one,
  // but you can also remove it
  {
    path: '/:catchAll(.*)*',
    component: () => import('pages/ErrorNotFound.vue'),
  },
];

export default routes;
