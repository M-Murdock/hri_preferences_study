from flask import Flask
from flask import render_template

app = Flask(__name__)

@app.route("/")
def hello_world():
    return render_template('index.html')

if __name__ == '__main__':  
    #app.run(host="10.42.99.24")
    app.run(host="10.42.99.11")
    # app.run(debug="true")