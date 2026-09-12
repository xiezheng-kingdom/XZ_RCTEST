/**
 ******************************************************************************
 * @file    cargo.c
 * @brief   地图导航仓储系统: 图路径规划 → 循线 → QR定位 → 机械臂
 *
 * 地图 (俯视, 左右对称于4-5轴, 星型拓扑):
 *      1──┐          ┌──6
 *         │          │
 *      2──9────4────10──7
 *         │    │     │
 *      3──┘    5     └──8
 *              ↑
 *            丁字路口
 *
 * 主干: 9-4-10   4=丁字(5接入)   9/10=T字
 * 边: 1-9, 3-9, 2-9, 9-4, 4-10, 10-7, 4-5, 10-6, 10-8
 * 每个节点有 QR 码, BFS 最短路径导航
 ******************************************************************************
 */
#include "cargo.h"
#include "hc06.h"
#include "road_map.h"
#include "nav.h"
#include "usart.h"
#include "ir_sensor.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* ================================================================
 * Nav 函数边表 (替代 RoadMap_GoStraight)
 * ================================================================ */
typedef uint8_t (*NavFunc)(void);
typedef struct { uint8_t from, to; NavFunc f; } NavEdge;
static const NavEdge nav_table[] = {
    {5,9,Nav_Turn4to9}, {5,10,Nav_Turn4to10},
    {4,9,Nav_10to9}, {9,4,Nav_10to9},   /* 4↔9 直行同10→9 */
    {4,10,Nav_10to9}, {10,4,Nav_10to9}, /* 4↔10 直行同10→9 */
    {4,5,Nav_9to2}, {5,4,Nav_10to7},    /* 4↔5 直行 */
    {9,1,Nav_9to1}, {9,2,Nav_9to2}, {9,3,Nav_9to3},
    {1,9,Nav_10to9}, {2,9,Nav_10to9}, {3,9,Nav_10to9}, /* 反向直行 */
    {10,6,Nav_Turn10to6}, {10,7,Nav_10to7}, {10,8,Nav_Turn10to8}, {10,9,Nav_10to9},
    {7,10,Nav_10to9}, {8,10,Nav_10to9}, /* 7/8→10 直行 */
    {6,10,Nav_6to10},
};
#define NAV_N (sizeof(nav_table)/sizeof(nav_table[0]))

static NavFunc g_nav_func = NULL;   /* 当前执行的nav函数 */

static NavFunc nav_lookup(uint8_t from, uint8_t to) {
    for (int i = 0; i < (int)NAV_N; i++)
        if (nav_table[i].from == from && nav_table[i].to == to)
            return nav_table[i].f;
    return NULL;
}

/* ================================================================
 * 邻接边表 (实测标定距离, 单位 cm)
 * ================================================================ */
const MapEdge g_map_edges[] = {
    /* 邻接边 + 距离(cm) + 转向提示
     * ⚠️ 实际转向由 CARGO 在路口 QR定位后根据当前航向+目标节点动态计算
     * ir_action 仅为提示 (假设从水平主路进入分支) */
    {4, 5, 30.5f, 0},                    /* 5↔4 */
    {9, 4, 24.8f, 0},  {4,10, 24.8f, 0}, /* 水平主干 */
    {2, 9, 25.7f, 0},  {10,7, 25.7f, 0}, /* 水平直连 */
    {1, 9, 49.0f, 2},  {3, 9, 49.0f, 1},  /* 9↔1(右) 9↔3(左) */
    {10,6, 49.0f, 1},  {10,8, 49.0f, 2},  /* 10↔6(左) 10↔8(右) */
};
const uint8_t g_map_edge_count = sizeof(g_map_edges) / sizeof(g_map_edges[0]);

/* 节点类型 */
/* 节点类型表 (供后续自动判定到达节点时的行为: 储物/取货/出货/路口) */
static const uint8_t g_node_type[NODE_MAX] = {
    [1]=NT_STORE, [2]=NT_STORE, [3]=NT_STORE,
    [4]=NT_CROSS, [9]=NT_CROSS, [10]=NT_CROSS,
    [5]=NT_HOME,
    [6]=NT_PICKUP,
    [7]=NT_FLEX,
    [8]=NT_DELIVER,
};

/* ================================================================
 * HC06 指令解析
 * ================================================================ */
#define RXBUF_LEN  64
static uint8_t  rx_buf[RXBUF_LEN];
static uint8_t  rx_idx = 0;
static uint8_t  rx_done = 0;

static CmdType  g_cmd_type;
static char     g_cmd_id[CARGO_ID_LEN];
static char     g_cmd_color[CARGO_COLOR_LEN];
static MapNode  g_cmd_shelf_node;   /* 入库目标储物台 */

/* ================================================================
 * 货物数据库
 * ================================================================ */
static CargoRecord g_db[CARGO_DB_MAX];
static uint8_t     g_db_count = 0;

/* ================================================================
 * 导航状态
 * ================================================================ */
static CargoState  g_state       = CS_IDLE;
static MapNode     g_current     = NODE_4;   /* 从路口4开始 */
static MapNode     g_target      = NODE_5;
static MapNode     g_path[8];               /* 规划路径 */
static uint8_t     g_path_len    = 0;
static uint8_t     g_path_idx    = 0;        /* 当前路径段索引 */
static uint32_t    g_wait_t0     = 0;

/* 电池 */
static int8_t      g_bat_pct     = 100;    /* 电池 % (模拟) */

/* 待执行指令 */
static uint8_t     g_pending     = 0;
static CmdType     g_pend_type;
static char        g_pend_id[CARGO_ID_LEN];
static char        g_pend_color[CARGO_COLOR_LEN];
static MapNode     g_pend_shelf;

/* 中断恢复 (路障处理) */
static CargoState  g_saved_state = CS_IDLE;
static CmdType     g_saved_cmd;
static MapNode     g_saved_target;
static MapNode     g_saved_path[8];
static uint8_t     g_saved_path_len;
static uint8_t     g_saved_path_idx;
static uint32_t    g_last_obstacle_scan = 0;

/* 机械臂 USART: UART7 */
#define ARM_UART  &huart7

/* ================================================================
 * 内部函数
 * ================================================================ */
static int  bfs_path(MapNode from, MapNode to, MapNode *path, uint8_t *len);
static int  db_find_by_id(const char *id);
static int  db_add(const char *id, const char *color, MapNode node,
                   float car_x, float car_y, float cgo_x, float cgo_y);
static void arm_send(const char *cmd);
static int  arm_wait_ok(uint32_t timeout_ms);
static int  k230_scan_qr(float *ox, float *oy);          /* 模式1 */
static void hc06_reply_ok(CmdType t, const char *id, MapNode node, float ox, float oy);
static int  k230_scan_cargo_match(const CargoRecord *rec);/* 模式2:货物QR匹配 */
static void hc06_reply_err(uint8_t code);
static uint8_t bfs_edge_count(MapNode from, MapNode to);
static int calc_task_cost(MapNode from, MapNode via, MapNode to);
static uint8_t is_endpoint(MapNode node);
static void parse_frame(void);

/* ================================================================
 * Cargo_Init
 * ================================================================ */
void Cargo_Init(void)
{
    memset(g_db, 0, sizeof(g_db));
    g_db_count = 0;
    g_state    = CS_IDLE;
    g_current  = NODE_4;
    g_target   = NODE_4;
    HC06_Init();
    HC06_SendString("[READY]\r\n");
}

/* ================================================================
 * Cargo_FeedByte
 * ================================================================ */
void Cargo_FeedByte(uint8_t byte)
{
    if (rx_done) return;
    if (byte == '[') { rx_idx = 0; return; }             /* 帧头 */
    if (byte == ']') { rx_buf[rx_idx] = '\0'; rx_done = 1; return; } /* 帧尾 */
    if (rx_idx < RXBUF_LEN - 1) rx_buf[rx_idx++] = byte;
    else { rx_idx = 0; HC06_SendString("[ERR,OVF]\r\n"); } /* 溢出 */
}

/* ================================================================
 * 解析 HC06 帧: $ST,id,color,shelf  /  $GT,id  /  $GO,node
 * ================================================================ */
static void parse_frame(void)
{
    g_cmd_type = CMD_NONE;
    g_cmd_shelf_node = NODE_1;
    if (rx_idx < 4) return;

    /* 格式: [CMD,arg1,arg2,...]  第一个字段=指令类型 */
    char *p = (char *)rx_buf;

    if      (strncmp(p, "ST,", 3) == 0) { g_cmd_type = CMD_STORE; p += 3; }
    else if (strncmp(p, "GT,", 3) == 0) { g_cmd_type = CMD_RETRIEVE; p += 3; }
    else if (strncmp(p, "GO,", 3) == 0) { g_cmd_type = CMD_GOTO; p += 3; }
    else if (strncmp(p, "CHG,",4) == 0) { g_cmd_type = CMD_CHARGE; p += 4; }
    else if (strncmp(p, "CONT",4) == 0) { g_cmd_type = CMD_CONTINUE; return; }
    else { HC06_SendString("[ERR,FMT]\r\n"); return; }

    if (g_cmd_type == CMD_STORE) {
        char *s1 = p;
        char *s2 = strchr(s1, ','); if (!s2) return; *s2 = '\0';
        char *s3 = strchr(s2 + 1, ','); if (!s3) return; *s3 = '\0';
        strncpy(g_cmd_id, s1, CARGO_ID_LEN - 1);
        strncpy(g_cmd_color, s2 + 1, CARGO_COLOR_LEN - 1);
        int sh = atoi(s3 + 1);
        g_cmd_shelf_node = (sh >= 1 && sh <= 3) ? (MapNode)sh : NODE_1;
        printf("CMD: STORE %s %s shelf=%d\r\n", g_cmd_id, g_cmd_color, sh);
    } else if (g_cmd_type == CMD_RETRIEVE) {
        strncpy(g_cmd_id, p, CARGO_ID_LEN - 1);
        printf("CMD: GET %s\r\n", g_cmd_id);
    } else {
        g_cmd_shelf_node = (MapNode)atoi(p);
        printf("CMD: GOTO/CHG node=%d\r\n", g_cmd_shelf_node);
    }
}

/* ================================================================
 * Cargo_Run — 主循环
 * ================================================================ */
void Cargo_Run(void)
{
    uint32_t now = HAL_GetTick();

    /* ---- 解析新指令 ---- */
    if (rx_done) {
        parse_frame();
        rx_idx = 0; rx_done = 0;
        if (g_cmd_type != CMD_NONE) {
            /* 预计算任务耗电 */
            int cost = 0;
            if (g_cmd_type == CMD_STORE)
                cost = calc_task_cost(g_current, NODE_6, g_cmd_shelf_node);
            else if (g_cmd_type == CMD_RETRIEVE)
                cost = bfs_edge_count(g_current, g_cmd_shelf_node) + bfs_edge_count(g_cmd_shelf_node, NODE_8);
            else if (g_cmd_type == CMD_GOTO)
                cost = bfs_edge_count(g_current, g_cmd_shelf_node) + bfs_edge_count(g_cmd_shelf_node, NODE_5);
            cost++; /* 返航多加1边 */

            int remain = g_bat_pct - cost;
            if (remain < BAT_LOW_PCT) {
                /* 低电: 拒绝, 存入pending, 等待充电 */
                g_pending = 1; g_pend_type = g_cmd_type;
                strncpy(g_pend_id, g_cmd_id, CARGO_ID_LEN-1);
                strncpy(g_pend_color, g_cmd_color, CARGO_COLOR_LEN-1);
                g_pend_shelf = g_cmd_shelf_node;
                char buf[32]; snprintf(buf, sizeof(buf), "[ERR,BAT,%d]\r\n", g_bat_pct);
                HC06_SendString(buf); g_cmd_type = CMD_NONE; return;
            }
            if (remain < BAT_MED_PCT) HC06_SendString("[BAT,WARN]\r\n");

            if (g_cmd_type == CMD_STORE) {
                bfs_path(g_current, NODE_6, g_path, &g_path_len);
                g_target = g_cmd_shelf_node;
                g_state  = CS_NAV_TO; g_path_idx = 0;
            } else if (g_cmd_type == CMD_RETRIEVE) {
                int idx = db_find_by_id(g_cmd_id);
                if (idx < 0) { hc06_reply_err(1); g_state = CS_IDLE; return; }
                g_cmd_shelf_node = g_db[idx].shelf_node;
                bfs_path(g_current, g_cmd_shelf_node, g_path, &g_path_len);
                g_target = NODE_8;
                g_state  = CS_NAV_TO; g_path_idx = 0;
            } else if (g_cmd_type == CMD_GOTO) {
                /* 直接调度: GO,10→Nav_Turn4to9 */
                if      (g_cmd_shelf_node == 9)  g_nav_func = (NavFunc)Nav_Turn4to9;
                else if (g_cmd_shelf_node == 10) g_nav_func = (NavFunc)Nav_Turn4to10;
                else g_nav_func = NULL;
                if (g_nav_func) g_state = CS_NAV_TO;
            } else if (g_cmd_type == CMD_CHARGE) {
                g_bat_pct = atoi(g_cmd_id); if (g_bat_pct > 100) g_bat_pct = 100;
                char buf[32]; snprintf(buf, sizeof(buf), "[OK,CHG,%d]\r\n", g_bat_pct);
                HC06_SendString(buf);
            } else if (g_cmd_type == CMD_CONTINUE) {
                if (g_state == CS_OBSTACLE_FAIL || g_state == CS_WAIT_RESUME) {
                    K230_SwitchToCargo(); HAL_Delay(50); K230_RequestCargo();
                    HAL_Delay(200);
                    if (Localization_IsCargoValid() && Localization_GetCargoCount() > 0) {
                        HC06_SendString("[OBS,STILL]\r\n"); /* 还有路障 */
                    } else {
                        g_state = CS_WAIT_RESUME; /* 清障完成, 恢复 */
                    }
                }
            }
            g_cmd_type = CMD_NONE; return;
        }
    }

    /* ---- 推进导航 (直接执行nav函数) ---- */
    if (g_state == CS_NAV_TO) {
        if (g_nav_func && g_nav_func()) {
            g_nav_func = NULL;
            g_state = CS_DONE;
        }
    }

    /* ---- 到达节点后的动作 (简化) ---- */
    if (g_state == CS_AT_NODE) {
        g_state = CS_DONE;
    }

    if (g_state == CS_ARM_WAIT) {
        if (arm_wait_ok(5000) || now - g_wait_t0 > 3000) {
            if (g_cmd_type == CMD_STORE) {
                /* 入库: 已从6取货, 去储物台放置 */
                bfs_path(g_current, g_cmd_shelf_node, g_path, &g_path_len);
                g_target = g_cmd_shelf_node;
                g_state  = CS_NAV_TO; g_path_idx = 0;
                g_cmd_type = CMD_NONE;
            } else {
                /* 出库等其他指令直接完成 */
                g_state = CS_DONE;
            }
        }
    }

    if (g_state == CS_DONE) {
        HC06_SendString("[NAV,DONE]");
        g_cmd_type = CMD_NONE;
        g_state = CS_IDLE;
    }

    /* ---- 返航: 直接完成 (TODO: nav返航函数) ---- */
    if (g_state == CS_RETURN) { g_state = CS_IDLE; }
    if (0 && g_state == CS_RETURN && RoadMap_IsDone() && g_path_len == 0) {
        bfs_path(g_current, NODE_5, g_path, &g_path_len);
        g_path_idx = 0;
    }

    /* ---- 返航 ---- */
    if (g_state == CS_RETURN) {
        if (g_path_idx < g_path_len && RoadMap_IsDone()) {
            /* 推进到路径下一段 */
            MapNode from = g_path[g_path_idx];
            MapNode to = (g_path_idx + 1 < g_path_len)
                         ? g_path[g_path_idx + 1] : NODE_5;
            /* 查边距 */
            float dist = 30.0f;
            for (int i = 0; i < g_map_edge_count; i++) {
                if ((g_map_edges[i].from == from && g_map_edges[i].to == to) ||
                    (g_map_edges[i].to == from && g_map_edges[i].from == to))
                    { dist = g_map_edges[i].dist_cm; break; }
            }
            RoadMap_GoStraight(dist, g_chassis.duty_run);
            g_current = to;
            g_path_idx++;
        }
        if (g_path_idx >= g_path_len && RoadMap_IsDone()) {
            /* 回到5, 掉头面向4 */
            RoadMap_Rotate(180.0f, g_chassis.duty_run);
            g_state = CS_RETURN_DONE;
        }
    }

    if (g_state == CS_RETURN_DONE && RoadMap_IsDone()) {
        /* 掉头完成 → 沿辅助线到路口4 */
        RoadMap_GoStraight(g_map_edges[0].dist_cm, g_chassis.duty_run);
        g_target = NODE_4; g_state = CS_NAV_TO;
    }

    /* 到达4后电池判断 */
    if (g_state == CS_NAV_TO && RoadMap_IsDone() && g_target == NODE_4
        && g_current == NODE_5) {
        g_current = NODE_4;
        /* 任务消耗电量 */
        uint8_t cost = bfs_edge_count(NODE_5, NODE_4) + 1;
        g_bat_pct -= cost; if (g_bat_pct < 0) g_bat_pct = 0;

        if (g_bat_pct >= BAT_MED_PCT)         { g_state = CS_IDLE; }
        else if (g_bat_pct >= BAT_LOW_PCT)    { HC06_SendString("[BAT,WARN]\r\n"); g_state = CS_IDLE; }
        else { HC06_SendString("[BAT,LOW]\r\n");
               RoadMap_GoStraight(g_map_edges[0].dist_cm, g_chassis.duty_run);
               g_target = NODE_5; g_state = CS_LOW_BATTERY; }
    }
    if (g_state == CS_LOW_BATTERY && RoadMap_IsDone()) {
        g_current = NODE_5; g_bat_pct -= 2; if (g_bat_pct < 0) g_bat_pct = 0;
    }

    /* 待执行指令恢复 (电量>=MED即可) */
    if (g_pending && g_bat_pct >= BAT_MED_PCT) {
        g_cmd_type = g_pend_type; strncpy(g_cmd_id, g_pend_id, CARGO_ID_LEN-1);
        strncpy(g_cmd_color, g_pend_color, CARGO_COLOR_LEN-1);
        g_cmd_shelf_node = g_pend_shelf; g_pending = 0;
        if (g_cmd_type == CMD_STORE) { bfs_path(g_current, NODE_6, g_path, &g_path_len);
            g_target = g_cmd_shelf_node; g_state = CS_NAV_TO; g_path_idx = 0; }
    }

    /* ---- 定时路障扫描 (导航中 / 返航中, 机械臂空载) ---- */
    if ((g_state == CS_NAV_TO || g_state == CS_RETURN) &&
        now - g_last_obstacle_scan >= 2000) {
        g_last_obstacle_scan = now;
        K230_SwitchToCargo(); HAL_Delay(30); K230_RequestCargo();
        /* 等50ms再看结果 (非阻塞检查) */
    }
    if ((g_state == CS_NAV_TO || g_state == CS_RETURN) &&
        Localization_IsCargoValid() && !g_pending) {
        /* 途中扫描到货物 → 路障! */
        uint8_t n = Localization_GetCargoCount();
        if (n > 0) {
            /* 保存中断点 */
            g_saved_state = g_state; g_saved_cmd = g_cmd_type;
            g_saved_target = g_target;
            memcpy(g_saved_path, g_path, sizeof(g_path));
            g_saved_path_len = g_path_len; g_saved_path_idx = g_path_idx;
            /* 停车拾取路障 */
            RoadMap_Stop();
            arm_send("[ARM,PICK]\r\n");
            g_wait_t0 = now; g_state = CS_OBSTACLE;
            HC06_SendString("[OBS,DETECT]\r\n");
        }
    }

    /* ---- 路障处理 ---- */
    if (g_state == CS_OBSTACLE) {
        if (arm_wait_ok(3000) || now - g_wait_t0 > 5000) {
            /* 再扫一次判断是否拾取成功 */
            K230_SwitchToCargo(); HAL_Delay(50); K230_RequestCargo();
            g_wait_t0 = now; g_state = CS_OBSTACLE_TO_7;
        }
    }
    if (g_state == CS_OBSTACLE_TO_7) {
        if (Localization_IsCargoValid() && Localization_GetCargoCount() > 0) {
            /* 拾取失败, 货物仍在 → 人工介入 */
            HC06_SendString("[OBS,FAIL,HELP]\r\n");
            g_state = CS_OBSTACLE_FAIL;
        } else {
            /* 拾取成功 → 转运到杂货台7 */
            bfs_path(g_current, NODE_7, g_path, &g_path_len);
            g_path_idx = 0; g_state = CS_NAV_TO; g_target = NODE_7;
            HC06_SendString("[OBS,CLEAR,TO7]\r\n");
            /* 到达7后继续: 在CS_DONE时特殊处理 */
        }
    }
    if (g_state == CS_OBSTACLE_FAIL) {
        /* 等待扫不到货物 + 收到 $CONT */
    }
    if (g_state == CS_WAIT_RESUME) {
        /* 恢复中断的导航 */
        g_state   = g_saved_state; g_cmd_type = g_saved_cmd;
        g_target  = g_saved_target;
        memcpy(g_path, g_saved_path, sizeof(g_path));
        g_path_len = g_saved_path_len; g_path_idx = g_saved_path_idx;
        HC06_SendString("[OBS,RESUME]\r\n");
        g_state = g_saved_state;
    }

    RoadMap_Run();
}

/* ================================================================
 * BFS 最短路径
 * ================================================================ */
static int bfs_path(MapNode from, MapNode to, MapNode *path, uint8_t *len)
{
    if (from == to) { path[0] = from; *len = 1; return 0; }

    int8_t visited[NODE_MAX] = {0};
    MapNode prev[NODE_MAX] = {0};
    MapNode queue[16];
    uint8_t head = 0, tail = 0;

    queue[tail++] = from;
    visited[from] = 1;

    while (head < tail) {
        MapNode u = queue[head++];
        for (int i = 0; i < g_map_edge_count; i++) {
            MapNode v = NODE_MAX;
            if (g_map_edges[i].from == u) v = g_map_edges[i].to;
            else if (g_map_edges[i].to == u) v = g_map_edges[i].from;
            if (v >= NODE_MAX) continue;

            if (!visited[v]) {
                visited[v] = 1;
                prev[v] = u;
                queue[tail++] = v;
                if (v == to) goto found;
            }
        }
    }
    return -1;  /* 无路径 */

found:
    *len = 0;
    MapNode cur = to;
    MapNode tmp[8]; uint8_t i = 0;
    while (cur != from) { tmp[i++] = cur; cur = prev[cur]; }
    tmp[i++] = from;
    for (uint8_t j = 0; j < i; j++) path[j] = tmp[i - 1 - j];
    *len = i;
    return 0;
}

/* ================================================================
 * 数据库
 * ================================================================ */
static int db_find_by_id(const char *id) {
    for (int i = 0; i < g_db_count; i++)
        if (g_db[i].stored && strcmp(g_db[i].id, id) == 0) return i;
    return -1;
}
static int db_add(const char *id, const char *color, MapNode node,
                   float car_x, float car_y, float cgo_x, float cgo_y) {
    if (g_db_count >= CARGO_DB_MAX) return -1;
    CargoRecord *r = &g_db[g_db_count];
    strncpy(r->id, id, CARGO_ID_LEN - 1);
    strncpy(r->color, color, CARGO_COLOR_LEN - 1);
    r->shelf_node = node;
    r->car_ox   = car_x;   r->car_oy   = car_y;
    r->cargo_ox = cgo_x;   r->cargo_oy = cgo_y;
    r->stored = 1;
    g_db_count++;
    printf("DB: +%s %s N%d car(%.1f,%.1f) cgo(%.1f,%.1f) [%d]\r\n",
           id, color, node, car_x, car_y, cgo_x, cgo_y, g_db_count);
    return 0;
}

/* ================================================================
 * 机械臂
 * ================================================================ */
static void arm_send(const char *cmd) {
    HAL_UART_Transmit(ARM_UART, (uint8_t *)cmd, (uint16_t)strlen(cmd), 500);
}
static int arm_wait_ok(uint32_t timeout_ms) {
    (void)timeout_ms; return 1;  /* TODO: 真RX */
}

/* K230 模式1: 扫描QR码, 返回当前小车相对位置 (通过 USART 接收) */
static int k230_scan_qr(float *ox, float *oy)
{
    /* TODO: 通过 LOCALIZATION 模块接收 K230 QR 数据
     * 协议: $QR,node,Xcm,Ycm,heading\r\n */
    if (Localization_IsDataValid()) {
        *ox = Localization_GetQR_X();
        *oy = Localization_GetQR_Y();
        return 1;
    }
    return 0;
}

/* K230 货物匹配: 基于不变向量 Δ, 从扫描列表中识别目标货物
 * Δ = (cargo_ox-car_ox, cargo_oy-car_oy) — 货物相对小车的偏移恒定
 * 当前QR位置 + Δ = 预期货物位置 → 与扫描列表匹配最近者
 * 返回: 0=匹配成功  -1=未找到  (待 K230 联调后启用) */
static int k230_scan_cargo_match(const CargoRecord *rec)
{
    if (!rec) return -1;
    float qr_x, qr_y;
    if (!k230_scan_qr(&qr_x, &qr_y)) return -1;
    float dx = rec->cargo_ox - rec->car_ox;
    float dy = rec->cargo_oy - rec->car_oy;
    float expect_x = qr_x + dx;
    float expect_y = qr_y + dy;
    /* TODO: 遍历货物扫描列表, 匹配与(expect_x,expect_y)最近的点 */
    (void)expect_x; (void)expect_y;
    return -1;
}

/* ================================================================
 * HC06 回复
 * ================================================================ */
static void hc06_reply_ok(CmdType t, const char *id, MapNode node, float ox, float oy) {
    char buf[64];
    const char *op = (t == CMD_STORE) ? "ST" : (t == CMD_RETRIEVE) ? "GT" : "GO";
    snprintf(buf, sizeof(buf), "[OK,%s,%s,N%d,%.1f,%.1f]\r\n", op, id, node, ox, oy);
    HC06_SendString(buf);
}
static void hc06_reply_err(uint8_t code) {
    char buf[32];
    snprintf(buf, sizeof(buf), "[ERR,%u]\r\n", code);
    HC06_SendString(buf);
}

/* ================================================================
 * 电池模拟: BFS路径边数×1% = 消耗, 阈值见 cargo.h
 * ================================================================ */
static uint8_t bfs_edge_count(MapNode from, MapNode to)
{
    MapNode tmp[8]; uint8_t len;
    if (bfs_path(from, to, tmp, &len) < 0) return 99;
    return (len > 0) ? len - 1 : 0;  /* N个节点 = N-1条边 */
}

static int calc_task_cost(MapNode from, MapNode via, MapNode to)
{
    return bfs_edge_count(from, via) + bfs_edge_count(via, to) + 1; /* +返航 */
}

uint8_t Cargo_GetBatteryPct(void) { return (uint8_t)g_bat_pct; }
uint8_t Cargo_IsBatteryLow(void)  { return g_bat_pct < BAT_LOW_PCT; }

/* 端点判定: 度数=1的节点 (1,2,3,5,6,7,8) */
static uint8_t is_endpoint(MapNode node)
{
    int deg = 0;
    for (int i = 0; i < g_map_edge_count; i++) {
        if (g_map_edges[i].from == node) deg++;
        if (g_map_edges[i].to   == node) deg++;
    }
    return (deg == 1) ? 1 : 0;
}

/* ================================================================
 * 查询
 * ================================================================ */
CargoState Cargo_GetState(void) { return g_state; }
void Cargo_GetStateStr(char *buf, uint8_t len) {
    const char *s[] = {"IDLE","NAV_TO","AT_NODE","ARM_WAIT","DONE",
        "RETURN","RET_DONE","LO_BAT","PENDING",
        "OBS","OBS_TO7","OBS_FAIL","WAIT_RSM","ERROR"};
    uint8_t idx = g_state;
    if (idx > CS_ERROR) idx = CS_ERROR;
    snprintf(buf, len, "%s", s[idx]);
}
MapNode Cargo_GetCurrentNode(void) { return g_current; }
MapNode Cargo_GetTargetNode(void)  { return g_target; }
uint8_t Cargo_GetDBCount(void)    { return g_db_count; }
