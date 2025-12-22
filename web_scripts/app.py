import rclpy

import uvicorn, fastapi

import routers.detector_param_router as detector_param_router


app = fastapi.FastAPI()

app.include_router(detector_param_router.router)   

@app.get("/", response_class=fastapi.responses.HTMLResponse)
async def get_index():
    # 读取 HTML 文件并返回
    return fastapi.responses.FileResponse(path="templates/index.html", status_code=200)


def main(args=None):
    uvicorn.run(app, port=8000, log_level='warning')
    rclpy.shutdown()


if __name__ == '__main__':
    main()