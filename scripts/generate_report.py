import os
import subprocess
from google import genai

# Use Gemini 2.0 Flash for technical reasoning
client = genai.Client(api_key=os.environ["GEMINI_API_KEY"])

def get_git_changes():
    # Gets the diff of the 'src' folder from 1 month ago to now
    try:
        return subprocess.check_output(
            ['git', 'diff', 'HEAD@{1.month.ago}', 'HEAD', '--', 'src/'], 
            shell=True
        ).decode('utf-8')
    except:
        return "Initial report generation or no recent changes found."

def main():
    diff_content = get_git_changes()
    
    # Read the previous report for context (if it exists)
    previous_report = ""
    if os.path.exists("report/updates.tex"):
        with open("report/updates.tex", "r") as f:
            previous_report = f.read()

    prompt = f"""
    Context: You are an expert Robotics Engineer.
    Task: Compare recent code changes with the previous month's report and write an update.
    
    PREVIOUS REPORT CONTENT:
    {previous_report}
    
    GIT DIFF (LAST 30 DAYS):
    {diff_content}
    
    INSTRUCTIONS:
    1. Summarize hardware design changes, control logic updates, and sensor integration.
    2. Format the output in RAW LaTeX (no preamble).
    3. Use technical engineering terminology (mechatronics, kinematics, etc.).
    4. Provide only the updated LaTeX content.
    """

    response = client.models.generate_content(
        model="gemini-2.0-flash",
        contents=prompt
    )

    # Ensure the report directory exists
    os.makedirs("report", exist_ok=True)
    with open("report/updates.tex", "w") as f:
        f.write(response.text)

if __name__ == "__main__":
    main()