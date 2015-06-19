#pragma once

struct alpha_msg {
sns_msg_header header;
int type;
char line[200];
};
