from fastapi import FastAPI

app = FastAPI()

@app.get("/")
def read_root():
    return {"Hello": "World"}

@app.get("/pi/")
def communicate_with_pi():
    # Placeholder: Add code to communicate with your Raspberry Pi here
    return {"message": "Data from Raspberry Pi"}
