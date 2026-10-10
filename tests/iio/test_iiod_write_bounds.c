/* Focused source regression for IIO daemon attribute-write bounds. */
#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include "../../iio/iiod.c"

static uint32_t received_bytes;

static int fake_recv(struct iiod_ctx *ctx, uint8_t *buf, uint32_t len)
{
	(void)ctx;
	memset(buf, 0x41, len);
	received_bytes += len;
	return (int)len;
}

int main(void)
{
	struct iiod_desc desc = {0};
	struct iiod_conn_data conn_data = {0};
	struct iiod_conn_data removed = {0};
	struct iiod_conn_priv *conn;
	char payload[5] = {0, 0, 0, 0, (char)0x5a};
	uint32_t conn_id = UINT32_MAX;

	assert(iiod_conn_add(NULL, &conn_data, &conn_id) == -EINVAL);
	assert(iiod_conn_add(&desc, NULL, &conn_id) == -EINVAL);
	assert(iiod_conn_add(&desc, &conn_data, &conn_id) == -EINVAL);
	conn_data.buf = payload;
	conn_data.len = sizeof(payload) - 1;
	assert(iiod_conn_add(&desc, &conn_data, &conn_id) == 0);
	assert(iiod_conn_remove(&desc, IIOD_MAX_CONNECTIONS, &removed) == -EINVAL);
	assert(iiod_conn_remove(&desc, conn_id, NULL) == -EINVAL);

	desc.ops.recv = fake_recv;
	conn = &desc.conns[conn_id];
	conn->state = IIOD_READING_WRITE_DATA;
	conn->payload_buf = payload;
	conn->payload_buf_len = sizeof(payload) - 1;
	conn->discard_write_payload = true;
	conn->write_payload_remaining = 6;
	conn->nb_buf.buf = payload;
	conn->nb_buf.len = sizeof(payload) - 1;
	conn->cmd_data.cmd = IIOD_CMD_WRITE;

	assert(iiod_run_state(&desc, conn) == 0);
	assert(conn->state == IIOD_READING_WRITE_DATA);
	assert(conn->write_payload_remaining == 2);
	assert(conn->nb_buf.len == 2 && conn->nb_buf.idx == 0);
	assert(iiod_run_state(&desc, conn) == 0);
	assert(conn->state == IIOD_WRITING_CMD_RESULT);
	assert((int32_t)conn->res.val == -E2BIG);
	assert(conn->res.write_val);
	assert(received_bytes == 6);
	assert(payload[sizeof(payload) - 1] == (char)0x5a);

	return 0;
}
